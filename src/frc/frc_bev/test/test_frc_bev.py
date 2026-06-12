"""frc_bev 回归测试：合成数据张量行为断言。

固定 bag 片段回归（git-lfs）在 Jetson 数据就位后补充（Phase 1.1），
本文件先用确定性合成数据锁住 builder/labels/patches 的行为。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from frc_bev.builder import BevBuilder, SE2
from frc_bev.config import BevConfig
from frc_bev.labels import IGNORE, LabelConfig, label_event, label_negative
from frc_bev.patches import (PcaWhitener, PrototypeRetriever,
                             handcrafted_features, split_patches)

CFG = BevConfig()
HEALTH = np.zeros(len(CFG.health_keys), dtype=np.float32)


def make_builder_with_wall(x_wall=2.0):
    """在机器人正前方 x_wall 米放一面 0.3 m 高的墙。"""
    builder = BevBuilder(CFG)
    ys = np.linspace(-1.0, 1.0, 50)
    zs = np.linspace(-0.3, 0.3, 13)
    yy, zz = np.meshgrid(ys, zs)
    wall = np.stack([np.full(yy.size, x_wall), yy.ravel(), zz.ravel()], axis=1)
    builder.push_cloud(wall.astype(np.float32), stamp=10.0)
    return builder


class TestBevConfig:
    def test_grid_size(self):
        assert CFG.grid_size == 120
        assert CFG.num_channels == 7

    def test_fingerprint_stable(self):
        assert CFG.fingerprint() == BevConfig().fingerprint()

    def test_fingerprint_changes_with_upstream_crop(self):
        other = BevConfig(upstream_crop=(-0.5, 0.5))
        assert other.fingerprint() != CFG.fingerprint()

    def test_z_window_matches_upstream(self):
        # F5：BEV z 窗口必须覆盖且贴近上游裁剪 [-0.33, 0.30]
        assert CFG.z_min <= CFG.upstream_crop[0]
        assert CFG.z_max >= CFG.upstream_crop[1]


class TestBevBuilder:
    def test_empty_build(self):
        frame = BevBuilder(CFG).build(SE2(0, 0, 0), None, HEALTH, stamp=0.0)
        assert frame.tensor.shape == (7, 120, 120)
        unknown = frame.tensor[CFG.channels.index("unknown")]
        assert float(np.asarray(unknown, dtype=np.float32).min()) == 1.0

    def test_wall_appears_in_front(self):
        frame = make_builder_with_wall().build(SE2(0, 0, 0), None, HEALTH, 10.0)
        max_z = np.asarray(frame.tensor[CFG.channels.index("max_z")], np.float32)
        # 墙在 x=+2m：行 = (2+6)/0.1 = 80
        assert max_z[78:83, 45:75].max() == pytest.approx(0.3, abs=0.05)
        # 后方无点
        assert max_z[:60].max() == 0.0

    def test_heading_alignment(self):
        # 机器人朝 +y（yaw=90°）时，odom 系 +y 方向的墙应出现在 BEV 正前方
        builder = BevBuilder(CFG)
        ys = np.full(100, 3.0)
        xs = np.linspace(-1.0, 1.0, 100)
        wall = np.stack([xs, ys, np.full(100, 0.2)], axis=1)
        builder.push_cloud(wall.astype(np.float32), stamp=0.0)
        frame = builder.build(SE2(0, 0, np.pi / 2), None, HEALTH, 0.0)
        max_z = np.asarray(frame.tensor[CFG.channels.index("max_z")], np.float32)
        assert max_z[88:92].max() > 0.1     # x=+3m -> 行 90
        assert max_z[:70].max() == 0.0

    def test_agg_window_eviction(self):
        builder = BevBuilder(CFG)
        builder.push_cloud(np.array([[1.0, 0, 0]], dtype=np.float32), stamp=0.0)
        builder.push_cloud(np.array([[2.0, 0, 0]], dtype=np.float32), stamp=10.0)
        # 1.5s 窗口下 stamp=0 的帧已被裁掉
        assert len(builder._clouds) == 1

    def test_z_crop(self):
        builder = BevBuilder(CFG)
        pts = np.array([[1.0, 0, 1.0], [1.0, 0, 0.1]], dtype=np.float32)
        builder.push_cloud(pts, stamp=0.0)
        frame = builder.build(SE2(0, 0, 0), None, HEALTH, 0.0)
        max_z = np.asarray(frame.tensor[CFG.channels.index("max_z")], np.float32)
        assert max_z.max() == pytest.approx(0.1, abs=0.01)  # z=1.0 被裁

    def test_health_dim_check(self):
        with pytest.raises(ValueError):
            BevBuilder(CFG).build(SE2(0, 0, 0), None, np.zeros(2), 0.0)

    def test_deterministic_hash(self):
        f1 = make_builder_with_wall().build(SE2(0, 0, 0), None, HEALTH, 10.0)
        f2 = make_builder_with_wall().build(SE2(0, 0, 0), None, HEALTH, 10.0)
        assert f1.tensor.tobytes() == f2.tensor.tobytes()


class TestLabels:
    def test_positive_swath(self):
        traj = np.array([[-2.0, 0.0], [-1.0, 0.0], [0.0, 0.0]])
        bev = np.zeros((7, 120, 120), dtype=np.float32)
        bev[0, 60, 60] = 0.3  # max_z 凸起 -> 通过归因闸门
        label, weight = label_event((0.0, 0.0), traj, "gold", bev, CFG.channels,
                                    lio_min_eig=200.0, cfg_bev=CFG)
        assert label[60, 60] == 1
        assert float(weight[60, 60]) > 0.5
        # swath 外远处为负
        assert label[10, 10] == 0

    def test_attribution_gate(self):
        # LIO 退化 + swath 内无几何异常 -> 全图 ignore
        traj = np.array([[-1.0, 0.0], [0.0, 0.0]])
        bev = np.zeros((7, 120, 120), dtype=np.float32)
        label, weight = label_event((0.0, 0.0), traj, "silver", bev, CFG.channels,
                                    lio_min_eig=10.0, cfg_bev=CFG)
        assert (label == IGNORE).all()
        assert float(np.asarray(weight, np.float32).sum()) == 0.0

    def test_degenerate_with_geometry_still_labels(self):
        # LIO 退化但 swath 内确有凸起 -> 不触发闸门
        traj = np.array([[-1.0, 0.0], [0.0, 0.0]])
        bev = np.zeros((7, 120, 120), dtype=np.float32)
        bev[0, 60, 60] = 0.3
        label, _ = label_event((0.0, 0.0), traj, "silver", bev, CFG.channels,
                               lio_min_eig=10.0, cfg_bev=CFG)
        assert (label == 1).any()

    def test_negative_and_hard_negative(self):
        traj = np.array([[0.0, 0.0], [1.0, 0.0]])
        label, weight = label_negative(traj, CFG)
        assert label[60, 60] == 0
        assert float(weight[60, 60]) == 1.0
        _, hard_weight = label_negative(traj, CFG, hard=True)
        assert float(hard_weight[60, 60]) == 1.5


class TestPatches:
    def test_split_grid(self):
        tensor = np.zeros((7, 120, 120), dtype=np.float32)
        patches, centers = split_patches(tensor, CFG)
        # (120-40)/20+1 = 5 -> 25 patch
        assert patches.shape == (25, 7, 40, 40)
        assert centers.shape == (25, 2)

    def test_retrieval_pipeline(self):
        rng = np.random.default_rng(7)
        risky = rng.normal(1.0, 0.1, (20, 7, 40, 40)).astype(np.float32)
        safe = rng.normal(-1.0, 0.1, (20, 7, 40, 40)).astype(np.float32)
        feats = handcrafted_features(np.concatenate([risky, safe]), CFG.channels)
        whitener = PcaWhitener(dim=8).fit(feats)
        emb = whitener.transform(feats)
        retriever = PrototypeRetriever(emb[:20], np.ones(20), k=3,
                                       similarity_floor=0.3)
        risk_scores = retriever.score(emb[:20])
        safe_scores = retriever.score(emb[20:])
        assert risk_scores.mean() > safe_scores.mean() + 0.3

    def test_whitener_roundtrip(self):
        rng = np.random.default_rng(3)
        feats = rng.normal(size=(50, 30))
        w = PcaWhitener(dim=8).fit(feats)
        w2 = PcaWhitener.from_dict(w.to_dict())
        np.testing.assert_allclose(w.transform(feats), w2.transform(feats),
                                   rtol=1e-6)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
