SHELL := /bin/bash

# ==============================================================================
# Variables & Configuration
# ==============================================================================

# ROS 2 Environment and Build Flags
ROS_SETUP    := source /opt/ros/humble/setup.bash
PY_WARN_HIDE := PYTHONWARNINGS="ignore:Unknown distribution option:UserWarning"
COLCON_BUILD := $(ROS_SETUP) && $(PY_WARN_HIDE) colcon build --symlink-install --parallel-workers 1

# Massive regex for killing processes
POST_COLLISION_KILL_PATTERN := '[p]ost_collision_cmd_conditioner(\\.py)?'
PRIOR_MAP_TF_KILL_PATTERN := '[p]rior_map_tf_authority(\\.py)?'
KILL_PATTERN := '[l]aunch_with_logs.sh|[r]os2 launch|[m]onitor_corridor_status(\.py)?|[r]os2 bag|[r]viz2|[f]oxglove_bridge|[f]oxglove_navigation_adapter_node|[l]ivox_ros_driver2_node|[l]io_node|[l]ocalizer_node|[i]nitialpose_relocalize_bridge(\\.py)?|[n]av2_cloud_retime(\\.py)?|[l]ocalization_cmd_gate(\\.py)?|[c]ollision_monitor|[i]ndoor_navigation_manager_node|[p]go_node|[r]tk_fgo_node|[r]tk_map_odom_corrector|[s]erial_twistctl_node|[s]erial_reader_node|[n]mea_serial_driver|[u]m982_rtk_node|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[s]moother_server|[v]elocity_smoother|[l]ifecycle_manager|[w]aypoint_follower|[m]ap_server|[a]mcl|[c]omponent_container(_mt)?|[g]ps_route_runner|[g]ps_global_aligner|[g]ps_anchor_localizer|[r]oute_server|[g]oal_manager_node|[r]obot_state_publisher|[j]oint_state_publisher|[p]ointcloud_to_laserscan|[a]sync_slam_toolbox_node|[m]ap_saver_server|[f]rc_health_aggregator|[f]rc_event_marker|[f]rc_risk_pipeline|[f]rc_memory_manager|[f]rc_trial_runner|[s]urvey_node'

.PHONY: setup build build-% launch-% kill kill-runtime clean ntrip-% frc-daily test

# ==============================================================================
# Auto-complete Helpers (Empty targets to trick bash/zsh tab-completion)
# ==============================================================================
launch-slam launch-explore launch-indoor-nav launch-corridor launch-explore-gps launch-nav-gps launch-rtk-basic launch-tightly-coupled launch-travel launch-survey:

ntrip-logout ntrip-status ntrip-setup:

# ==============================================================================
# Setup & Clean
# ==============================================================================

setup:
	@echo ">>> 拉取第三方依赖..."
	git config --global --unset http.proxy || true
	git config --global --unset https.proxy || true
	vcs import < dependencies.repos
	@mkdir -p src/third_party/navigation2/nav2_system_tests
	@touch src/third_party/navigation2/nav2_system_tests/COLCON_IGNORE
	@echo ">>> 安装 rosdep 依赖..."
	rosdep install --from-paths src --ignore-src -y --skip-keys "slam_toolbox navigation2"
	@echo ">>> 环境配置完成"

clean:
	rm -rf build/ install/ log/

# ==============================================================================
# Build Targets
# ==============================================================================

build:
	$(COLCON_BUILD)

build-bringup:
	$(COLCON_BUILD) --packages-select bringup

build-fastlio2:
	$(COLCON_BUILD) --packages-select bringup fastlio2

build-rtk-basic:
	$(COLCON_BUILD) --packages-select serial nmea_msgs um982_rtk_driver

build-sensor:
	$(COLCON_BUILD) --packages-select \
		frc_msgs livox_ros_driver2 wit_ros2_imu wit_imu_traj \
		serial serial_reader serial_twistctl gyro_odometry \
		nmea_msgs nmea_navsat_driver um982_rtk_driver gnss_calibration wheeltec_gps_path

build-perception:
	$(COLCON_BUILD) --packages-select \
		frc_msgs fastlio2 hba localizer interface pgo pgo_original \
		pointcloud_to_laserscan pointcloud_to_grid rtk_fgo_localizer

build-planning:
	$(COLCON_BUILD) --packages-select global2local_tf gnss_global_path_planner global_path_planning

build-navigation:
	$(COLCON_BUILD) --packages-select waypoint_collector waypoint_nav_tool gps_waypoint_dispatcher indoor_navigation_manager

build-survey:
	$(COLCON_BUILD) --packages-select survey_mode

build-frc:
	$(COLCON_BUILD) --packages-select frc_msgs frc_bev frc_nodes_cpp frc_nodes frc_costmap_layer frc_bringup

# ==============================================================================
# Launch & Test Targets
# ==============================================================================

# Pattern rule: captures 'launch-slam', 'launch-explore', etc. and passes the suffix to the script.
launch-%:
	bash scripts/launch_with_logs.sh $*

test:
	$(ROS_SETUP) && colcon test && colcon test-result --verbose

# ==============================================================================
# Operations & Scripts
# ==============================================================================

# 用法: make frc-daily BAG=<rosbag2目录>
frc-daily:
	@test -n "$(BAG)" || (echo "用法: make frc-daily BAG=<rosbag2目录>"; exit 1)
	python3 -m frc_offline.failure_miner --bag $(BAG)
	python3 -m frc_offline.contact_sheet --bag $(BAG) --events $(BAG)/events.jsonl --out $(BAG)/review
	@echo ">>> 人工复核 $(BAG)/review/review.csv 后执行:"
	@echo ">>> python3 -m frc_offline.auto_label_from_events --events $(BAG)/events.jsonl --review $(BAG)/review/review.csv"

# Pattern rule for ntrip commands.
# Exception for ntrip-login which has no flag.
ntrip-login:
	@python3 scripts/setup_ntrip.py

ntrip-%:
	@python3 scripts/setup_ntrip.py --$*

# ==============================================================================
# Process Management
# ==============================================================================

kill: kill-runtime

kill-runtime:
	pkill -INT -f $(POST_COLLISION_KILL_PATTERN) || true
	pkill -INT -f $(PRIOR_MAP_TF_KILL_PATTERN) || true
	pkill -INT -f $(KILL_PATTERN) || true
	sleep 2
	pkill -KILL -f $(POST_COLLISION_KILL_PATTERN) || true
	pkill -KILL -f $(PRIOR_MAP_TF_KILL_PATTERN) || true
	pkill -KILL -f $(KILL_PATTERN) || true
	ros2 daemon stop >/dev/null 2>&1 || true
	@for dev in /dev/serial_twistctl /dev/wheeltec_gps /dev/rtk_um982; do \
		if [ -e "$$dev" ] && fuser "$$dev" >/dev/null 2>&1; then \
			fuser -k "$$dev" >/dev/null 2>&1 || true; \
		fi; \
	done
	@echo ">>> 导航相关残留进程已清理，ROS 2 daemon 已停止"
