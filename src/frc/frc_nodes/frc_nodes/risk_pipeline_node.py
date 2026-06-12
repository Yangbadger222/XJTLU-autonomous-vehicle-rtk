"""风险管线节点：BEV 构建 + 地图锚渲染 + 特征锚检索 + (v2)模型推理 + max 融合。

进程内 numpy 直传（feature_builder 与 risk 合并单进程，10 Hz 张量不走 DDS）。
输出 /frc/risk_grid 以 odom 系发布（与 local_costmap 同系），由本节点在发布时
用当下 map->odom TF 解算地图锚，frc_costmap_layer 内部完全不做 TF。

降级链：tick 超时跳帧告警 -> 连续超时停发 risk_grid -> frc_layer watchdog
0.5 s 自动旁路 -> Nav2 回到纯几何基线。任何 FRC 故障的最终形态都等价于没装。

三方校验：prototypes / 模型 sidecar 中的 bev_fingerprint 与本节点 BevConfig
指纹不一致时 FATAL 拒载（shadow 模式同样拒载）。
"""

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from frc_bev import BevBuilder, BevConfig, ros_adapter
from frc_bev.builder import SE2
from frc_bev.patches import (PcaWhitener, PrototypeRetriever,
                             handcrafted_features, split_patches)
from frc_msgs.msg import AnchorStateArray, Health, RiskGrid
from frc_nodes import session

try:
    from nav2_msgs.msg import Costmap
    HAVE_NAV2_MSGS = True
except ImportError:
    HAVE_NAV2_MSGS = False


class TrtRiskModel:
    """v2 TensorRT 推理薄封装。engine 缺失/TRT 不可用时禁用，不影响 v0/v1。"""

    def __init__(self, engine_path: str, logger):
        self.ok = False
        try:
            import tensorrt as trt  # noqa: F401  Jetson 端可用
            import pycuda.autoinit  # noqa: F401
            import pycuda.driver as cuda

            self._cuda = cuda
            trt_logger = trt.Logger(trt.Logger.WARNING)
            with open(engine_path, "rb") as f, trt.Runtime(trt_logger) as rt:
                self.engine = rt.deserialize_cuda_engine(f.read())
            self.context = self.engine.create_execution_context()
            self.ok = True
            logger.info(f"TRT engine loaded: {engine_path}")
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"TRT model disabled: {exc}")

    def infer(self, bev: np.ndarray, health: np.ndarray):
        """bev float16 [C,H,W] -> (risk [H,W] in [0,1], conf [H,W])，失败返回 None。"""
        if not self.ok:
            return None
        try:
            cuda = self._cuda
            inp = np.ascontiguousarray(
                np.concatenate([bev.astype(np.float32).ravel(),
                                health.astype(np.float32).ravel()]))
            h, w = bev.shape[1], bev.shape[2]
            out = np.empty(2 * h * w, dtype=np.float32)
            d_in = cuda.mem_alloc(inp.nbytes)
            d_out = cuda.mem_alloc(out.nbytes)
            cuda.memcpy_htod(d_in, inp)
            self.context.execute_v2([int(d_in), int(d_out)])
            cuda.memcpy_dtoh(out, d_out)
            risk = 1.0 / (1.0 + np.exp(-out[:h * w].reshape(h, w)))
            conf = np.clip(out[h * w:].reshape(h, w), 0.0, 1.0)
            return risk, conf
        except Exception:  # noqa: BLE001
            return None


class RiskPipelineNode(Node):
    def __init__(self):
        super().__init__("frc_risk_pipeline")
        p = self.declare_parameter
        p("tick_rate_hz", 10.0)
        p("tick_budget_ms", 80.0)
        p("overrun_disable_count", 5)
        p("shadow_mode", True)
        p("debug_tap", False)
        p("anchor_sigma_m", 0.75)
        p("feature_sigma_m", 1.0)
        p("feature_min_score", 0.15)
        p("prototypes_path", str(session.frc_dir("models") / "prototypes.npz"))
        p("model_engine_path", "")

        self.cfg = BevConfig()
        self.builder = BevBuilder(self.cfg)
        self._session = session.session_id()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose = None            # SE2 odom
        self._pose_stamp = 0.0
        self._costmap = None         # (grid, meta)
        self._health = np.zeros(len(self.cfg.health_keys), dtype=np.float32)
        self._anchors = []           # AnchorState list（map 系，memory_manager 解算）
        self._overruns = 0
        self._publishing = True

        self._retriever, self._whitener = self._load_prototypes()
        engine = str(self.get_parameter("model_engine_path").value)
        self._model = self._load_model(engine) if engine else None

        self.create_subscription(Odometry, "/fastlio2/lio_odom",
                                 self._on_odom, 20)
        self.create_subscription(PointCloud2, "/fastlio2/body_cloud",
                                 self._on_cloud, 5)
        self.create_subscription(Health, "/frc/health", self._on_health, 10)
        self.create_subscription(AnchorStateArray, "/frc/anchor_states",
                                 self._on_anchors, 10)
        if HAVE_NAV2_MSGS:
            self.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                     self._on_costmap, 2)

        self._risk_pub = self.create_publisher(RiskGrid, "/frc/risk_grid", 5)
        self._viz_pub = self.create_publisher(OccupancyGrid,
                                              "/frc/risk_grid_viz", 2)
        self._debug_pub = self.create_publisher(OccupancyGrid,
                                                "/frc/bev_debug", 2)

        rate = float(self.get_parameter("tick_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"frc_risk_pipeline started: fingerprint={self.cfg.fingerprint()} "
            f"shadow={self.get_parameter('shadow_mode').value} "
            f"feature={'on' if self._retriever else 'off'} "
            f"model={'on' if self._model and self._model.ok else 'off'}")

    # ---------- 模型/原型加载（含三方指纹校验）----------

    def _load_prototypes(self):
        path = Path(str(self.get_parameter("prototypes_path").value))
        if not path.exists():
            self.get_logger().info(f"no prototypes at {path}, feature anchor off")
            return None, None
        data = np.load(path, allow_pickle=False)
        fp = str(data["bev_fingerprint"])
        if fp != self.cfg.fingerprint():
            self.get_logger().fatal(
                f"prototypes fingerprint {fp} != BevConfig "
                f"{self.cfg.fingerprint()}, refusing to load (shadow 同样拒载)")
            raise SystemExit(1)
        whitener = PcaWhitener.from_dict(json.loads(str(data["pca_json"])))
        retriever = PrototypeRetriever(
            data["embeddings"].astype(np.float32),
            data["severities"].astype(np.float32))
        self.get_logger().info(
            f"prototypes loaded: {len(data['embeddings'])} entries")
        return retriever, whitener

    def _load_model(self, engine_path: str):
        meta_path = Path(engine_path + ".meta.json")
        if not meta_path.exists():
            self.get_logger().fatal(f"model sidecar missing: {meta_path}")
            raise SystemExit(1)
        meta = json.loads(meta_path.read_text())
        if meta.get("bev_fingerprint") != self.cfg.fingerprint():
            self.get_logger().fatal(
                f"model fingerprint {meta.get('bev_fingerprint')} != "
                f"{self.cfg.fingerprint()}, refusing to load")
            raise SystemExit(1)
        return TrtRiskModel(engine_path, self.get_logger())

    # ---------- 输入 ----------

    def _on_odom(self, msg):
        x, y, yaw = ros_adapter.odometry_to_se2(msg)
        self._pose = SE2(x, y, yaw)
        self._pose_z = float(msg.pose.pose.position.z)
        self._pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_cloud(self, msg):
        if self._pose is None:
            return
        xyz = ros_adapter.pointcloud2_to_xyz(msg)
        if not len(xyz):
            return
        # body 系 -> odom 系（用最近 lio_odom 位姿；同周期发布，误差可忽略）。
        # z 存相对车体原点高度，保证 BEV z 窗口在 odom z 漂移下仍正确（F5）。
        p = self._pose
        cos_y, sin_y = math.cos(p.yaw), math.sin(p.yaw)
        out = np.empty_like(xyz)
        out[:, 0] = p.x + xyz[:, 0] * cos_y - xyz[:, 1] * sin_y
        out[:, 1] = p.y + xyz[:, 0] * sin_y + xyz[:, 1] * cos_y
        out[:, 2] = xyz[:, 2]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.builder.push_cloud(out, stamp)

    def _on_health(self, msg):
        self._health = np.array([
            msg.lio_min_eig, float(msg.lio_degenerate),
            float(msg.pgo_correcting), float(msg.rtk_status),
            msg.v, msg.w], dtype=np.float32)

    def _on_anchors(self, msg):
        self._anchors = list(msg.anchors)

    def _on_costmap(self, msg):
        self._costmap = ros_adapter.costmap_to_numpy(msg)

    # ---------- 风险渲染 ----------

    def _render_gaussian(self, risk, conf, src, cx, cy, weight, sigma,
                         confidence, source_bit, origin_xy, res):
        """在 odom 轴对齐栅格上叠加一个高斯核（max 融合）。"""
        n = risk.shape[0]
        r_cells = max(int(3.0 * sigma / res), 1)
        ix = int(math.floor((cx - origin_xy[0]) / res))
        iy = int(math.floor((cy - origin_xy[1]) / res))
        x0, x1 = max(ix - r_cells, 0), min(ix + r_cells + 1, n)
        y0, y1 = max(iy - r_cells, 0), min(iy + r_cells + 1, n)
        if x0 >= x1 or y0 >= y1:
            return
        xs = origin_xy[0] + (np.arange(x0, x1) + 0.5) * res
        ys = origin_xy[1] + (np.arange(y0, y1) + 0.5) * res
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        g = weight * np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2)
                            / (2.0 * sigma ** 2))
        patch = risk[y0:y1, x0:x1]
        better = g > patch
        patch[better] = g[better]
        conf_patch = conf[y0:y1, x0:x1]
        conf_patch[better] = confidence
        src_patch = src[y0:y1, x0:x1]
        src_patch[better] = source_bit

    def _warp_bev_to_odom(self, bev_img, pose, origin_xy, res, n):
        """BEV 局部系（行=x 前、列=y 左）稠密图 -> odom 轴对齐栅格（最近邻）。"""
        xs = origin_xy[0] + (np.arange(n) + 0.5) * res
        ys = origin_xy[1] + (np.arange(n) + 0.5) * res
        wx, wy = np.meshgrid(xs, ys, indexing="xy")
        dx, dy = wx - pose.x, wy - pose.y
        cos_y, sin_y = math.cos(pose.yaw), math.sin(pose.yaw)
        lx = dx * cos_y + dy * sin_y
        ly = -dx * sin_y + dy * cos_y
        half = self.cfg.size_m / 2.0
        ri = np.floor((lx + half) / self.cfg.resolution).astype(np.int64)
        ci = np.floor((ly + half) / self.cfg.resolution).astype(np.int64)
        m = bev_img.shape[0]
        valid = (ri >= 0) & (ri < m) & (ci >= 0) & (ci < m)
        out = np.zeros((n, n), dtype=np.float32)
        out[valid] = bev_img[ri[valid], ci[valid]]
        return out

    # ---------- 主循环 ----------

    def _tick(self):
        if self._pose is None:
            return
        t0 = time.monotonic()
        pose = self._pose
        n = self.cfg.grid_size
        res = self.cfg.resolution
        origin_xy = (pose.x - self.cfg.size_m / 2.0,
                     pose.y - self.cfg.size_m / 2.0)

        risk = np.zeros((n, n), dtype=np.float32)
        conf = np.zeros((n, n), dtype=np.float32)
        src = np.zeros((n, n), dtype=np.uint8)
        source_mask = 0

        # 1) BEV（特征/模型共用；地图锚不依赖 BEV）
        costmap, cm_meta = self._costmap if self._costmap else (None, None)
        bev = self.builder.build(pose, costmap, self._health,
                                 self._pose_stamp, cm_meta)

        # 2) 地图锚：map 系 -> odom 系（当下 TF），高斯渲染
        anchor_sigma = float(self.get_parameter("anchor_sigma_m").value)
        tf_mo = self._map_to_odom()
        if self._anchors and tf_mo is not None:
            tx, ty, tyaw = tf_mo
            cos_t, sin_t = math.cos(tyaw), math.sin(tyaw)
            for a in self._anchors:
                weight = self._injection_weight(a)
                if weight <= 0.0:
                    continue
                ox = tx + a.map_x * cos_t - a.map_y * sin_t
                oy = ty + a.map_x * sin_t + a.map_y * cos_t
                confidence = a.p_usable * (0.8 if a.state == "candidate" else 1.0)
                self._render_gaussian(risk, conf, src, ox, oy, weight,
                                      anchor_sigma, confidence,
                                      RiskGrid.SOURCE_MAP_ANCHOR,
                                      origin_xy, res)
                source_mask |= RiskGrid.SOURCE_MAP_ANCHOR

        # 3) 特征锚检索（v1）：patch -> 嵌入 -> kNN -> 高斯
        if self._retriever is not None:
            patches, centers = split_patches(
                bev.tensor.astype(np.float32), self.cfg)
            feats = handcrafted_features(patches, self.cfg.channels)
            emb = self._whitener.transform(feats)
            scores = self._retriever.score(emb)
            min_score = float(self.get_parameter("feature_min_score").value)
            sigma_f = float(self.get_parameter("feature_sigma_m").value)
            cos_y, sin_y = math.cos(pose.yaw), math.sin(pose.yaw)
            for (lx, ly), s in zip(centers, scores):
                if s < min_score:
                    continue
                wx = pose.x + lx * cos_y - ly * sin_y
                wy = pose.y + lx * sin_y + ly * cos_y
                self._render_gaussian(risk, conf, src, wx, wy, float(s),
                                      sigma_f, float(s),
                                      RiskGrid.SOURCE_FEATURE_RETRIEVAL,
                                      origin_xy, res)
                source_mask |= RiskGrid.SOURCE_FEATURE_RETRIEVAL

        # 4) v2 模型稠密推理
        if self._model is not None and self._model.ok:
            result = self._model.infer(bev.tensor, bev.health)
            if result is not None:
                m_risk = self._warp_bev_to_odom(result[0], pose, origin_xy,
                                                res, n)
                m_conf = self._warp_bev_to_odom(result[1], pose, origin_xy,
                                                res, n)
                better = m_risk > risk
                risk[better] = m_risk[better]
                conf[better] = m_conf[better]
                src[better] = RiskGrid.SOURCE_MODEL
                source_mask |= RiskGrid.SOURCE_MODEL

        # 5) 超时降级
        elapsed_ms = (time.monotonic() - t0) * 1e3
        budget = float(self.get_parameter("tick_budget_ms").value)
        if elapsed_ms > budget:
            self._overruns += 1
            self.get_logger().warn(
                f"tick overrun {elapsed_ms:.0f}ms > {budget:.0f}ms "
                f"({self._overruns} consecutive)")
            if self._overruns >= int(
                    self.get_parameter("overrun_disable_count").value):
                if self._publishing:
                    self.get_logger().error(
                        "consecutive overruns: risk_grid publishing disabled, "
                        "frc_layer watchdog will bypass")
                self._publishing = False
                return
        else:
            if not self._publishing:
                self.get_logger().info("tick recovered, publishing resumed")
            self._overruns = 0
            self._publishing = True

        if not self._publishing:
            return

        self._publish(risk, conf, src, source_mask, origin_xy, res, n, bev)

    def _injection_weight(self, a) -> float:
        """与 anchor_store.Anchor.injection_weight 同公式（消息侧镜像）。"""
        if a.state == "retired":
            return 0.0
        w = a.severity * a.p_usable
        if a.state == "stale":
            w *= 0.5
        return float(min(w, 1.0))

    def _map_to_odom(self):
        try:
            tf = self._tf_buffer.lookup_transform("odom", "map",
                                                  rclpy.time.Time())
            q = tf.transform.rotation
            return (tf.transform.translation.x, tf.transform.translation.y,
                    ros_adapter.quat_to_yaw(q.x, q.y, q.z, q.w))
        except Exception:
            return None

    def _make_grid(self, data01, origin_xy, res, n) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "odom"
        grid.info.resolution = res
        grid.info.width = n
        grid.info.height = n
        grid.info.origin.position.x = origin_xy[0]
        grid.info.origin.position.y = origin_xy[1]
        grid.info.origin.orientation.w = 1.0
        grid.data = ros_adapter.numpy_to_occupancy_grid_data(data01) \
            .ravel().tolist()
        return grid

    def _publish(self, risk, conf, src, source_mask, origin_xy, res, n, bev):
        msg = RiskGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.risk = self._make_grid(risk, origin_xy, res, n)
        msg.confidence = self._make_grid(conf, origin_xy, res, n)
        msg.source_mask = source_mask
        self._risk_pub.publish(msg)

        if self._viz_pub.get_subscription_count() > 0:
            self._viz_pub.publish(self._make_grid(risk, origin_xy, res, n))

        if bool(self.get_parameter("debug_tap").value) and \
                self._debug_pub.get_subscription_count() > 0:
            zi = self.cfg.channels.index("max_z")
            max_z = np.asarray(bev.tensor[zi], dtype=np.float32)
            norm = np.clip((max_z - self.cfg.z_min)
                           / (self.cfg.z_max - self.cfg.z_min), 0, 1)
            # BEV 局部图变换到 odom 再发，便于与 risk overlay 对齐
            warped = self._warp_bev_to_odom(norm, self._pose, origin_xy, res, n)
            self._debug_pub.publish(self._make_grid(warped, origin_xy, res, n))


def main(args=None):
    rclpy.init(args=args)
    node = RiskPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
