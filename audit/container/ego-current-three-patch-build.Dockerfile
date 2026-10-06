FROM ros2-go2:humble
USER root
SHELL ["/bin/bash", "-lc"]
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /tmp
COPY patches /vehicle-research/patches
COPY src/research_interfaces /vehicle-research/src/research_interfaces
RUN sed -i 's/^Types: deb deb-src$/Types: deb/' /etc/apt/sources.list.d/ros2.sources \
 && apt-get update \
 && apt-get install -y --no-install-recommends libboost-all-dev \
 && rm -rf /var/lib/apt/lists/*
RUN git --version && command -v colcon && test -f /opt/ros/humble/setup.bash
RUN git clone --filter=blob:none --branch develop https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2.git /tmp/ego-source \
 && cd /tmp/ego-source \
 && git checkout 7f5be6d4cee34871e85aa1f15285cfaf17b23877 \
 && git apply /vehicle-research/patches/ego_planner_2d/0001-vehicle-state-and-feasibility.patch \
 && git apply /vehicle-research/patches/ego_planner_2d/0002-vehicle-ros-timed-trajectory.patch \
 && git apply /vehicle-research/patches/ego_planner_2d/0003-clear-stale-plan-on-failure.patch
RUN mkdir -p /tmp/ws/src \
 && cp -a /tmp/ego-source/src/EgoPlanner-ROS2 /tmp/ws/src/EgoPlanner-ROS2 \
 && cp -a /vehicle-research/src/research_interfaces /tmp/ws/src/research_interfaces \
 && source /opt/ros/humble/setup.bash \
 && colcon --log-base /tmp/ws/log build --base-paths /tmp/ws/src --build-base /tmp/ws/build --install-base /tmp/ws/install --packages-select research_interfaces ego_planner --event-handlers console_direct+ --parallel-workers 1
