SHELL := /bin/bash

.PHONY: setup build build-sensor build-perception build-planning build-navigation build-frc frc-daily test launch-slam launch-explore launch-indoor-nav launch-corridor launch-explore-gps launch-nav-gps launch-rtk-basic launch-travel kill kill-runtime clean

setup:
	@echo ">>> 拉取第三方依赖..."
	git config --global --unset http.proxy || true
	git config --global --unset https.proxy || true
	vcs import < dependencies.repos
	@echo ">>> 安装 rosdep 依赖..."
	rosdep install --from-paths src --ignore-src -y --skip-keys "slam_toolbox navigation2"
	@echo ">>> 环境配置完成"

build:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1

build-sensor:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1 --packages-select \
		frc_msgs \
		livox_ros_driver2 wit_ros2_imu wit_imu_traj \
		serial serial_reader serial_twistctl gyro_odometry \
		nmea_msgs nmea_navsat_driver um982_rtk_driver gnss_calibration wheeltec_gps_path

build-perception:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1 --packages-select \
		frc_msgs \
		fastlio2 hba localizer interface pgo pgo_original \
		pointcloud_to_laserscan pointcloud_to_grid

build-planning:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1 --packages-select \
		global2local_tf gnss_global_path_planner global_path_planning

build-navigation:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1 --packages-select \
		waypoint_collector waypoint_nav_tool gps_waypoint_dispatcher

build-frc:
	source /opt/ros/humble/setup.bash && \
	colcon build --symlink-install --parallel-workers 1 --packages-select \
		frc_msgs frc_bev frc_nodes_cpp frc_nodes frc_costmap_layer frc_bringup

# 工作站每日数据闭环（设计文档 §5.6）：miner -> contact_sheet -> 人工复核 ->
# auto_label。用法: make frc-daily BAG=<rosbag2目录>
frc-daily:
	@test -n "$(BAG)" || (echo "用法: make frc-daily BAG=<rosbag2目录>"; exit 1)
	python3 -m frc_offline.failure_miner --bag $(BAG)
	python3 -m frc_offline.contact_sheet --bag $(BAG) --events $(BAG)/events.jsonl --out $(BAG)/review
	@echo ">>> 人工复核 $(BAG)/review/review.csv 后执行:"
	@echo ">>> python3 -m frc_offline.auto_label_from_events --events $(BAG)/events.jsonl --review $(BAG)/review/review.csv"

test:
	source /opt/ros/humble/setup.bash && \
	colcon test && colcon test-result --verbose

launch-slam:
	bash scripts/launch_with_logs.sh slam

launch-explore:
	bash scripts/launch_with_logs.sh explore

launch-indoor-nav:
	bash scripts/launch_with_logs.sh indoor-nav

launch-corridor:
	bash scripts/launch_with_logs.sh corridor

launch-explore-gps:
	bash scripts/launch_with_logs.sh explore-gps

launch-nav-gps:
	bash scripts/launch_with_logs.sh nav-gps

launch-rtk-basic:
	bash scripts/launch_with_logs.sh rtk-basic

launch-travel:
	bash scripts/launch_with_logs.sh travel

kill:
	@$(MAKE) kill-runtime

kill-runtime:
	pkill -INT -f '[l]aunch_with_logs.sh|[m]onitor_corridor_status(\.py)?|[r]os2 bag|[r]viz2|[l]ivox_ros_driver2_node|[l]io_node|[p]go_node|[s]erial_twistctl_node|[n]mea_serial_driver|[u]m982_rtk_node|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[s]moother_server|[v]elocity_smoother|[l]ifecycle_manager|[w]aypoint_follower|[m]ap_server|[a]mcl|[c]omponent_container(_mt)?|[g]ps_route_runner|[g]ps_global_aligner|[r]obot_state_publisher|[p]ointcloud_to_laserscan|[f]rc_health_aggregator|[f]rc_event_marker|[f]rc_risk_pipeline|[f]rc_memory_manager|[f]rc_trial_runner' || true
	sleep 2
	pkill -KILL -f '[l]aunch_with_logs.sh|[m]onitor_corridor_status(\.py)?|[r]os2 bag|[r]viz2|[l]ivox_ros_driver2_node|[l]io_node|[p]go_node|[s]erial_twistctl_node|[n]mea_serial_driver|[u]m982_rtk_node|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[s]moother_server|[v]elocity_smoother|[l]ifecycle_manager|[w]aypoint_follower|[m]ap_server|[a]mcl|[c]omponent_container(_mt)?|[g]ps_route_runner|[g]ps_global_aligner|[r]obot_state_publisher|[p]ointcloud_to_laserscan|[f]rc_health_aggregator|[f]rc_event_marker|[f]rc_risk_pipeline|[f]rc_memory_manager|[f]rc_trial_runner' || true
	ros2 daemon stop >/dev/null 2>&1 || true
	@for dev in /dev/serial_twistctl /dev/wheeltec_gps /dev/rtk_um982; do \
		if [ -e "$$dev" ] && fuser "$$dev" >/dev/null 2>&1; then \
			fuser -k "$$dev" >/dev/null 2>&1 || true; \
		fi; \
	done
	@echo ">>> 导航相关残留进程已清理，ROS 2 daemon 已停止"

clean:
	rm -rf build/ install/ log/
