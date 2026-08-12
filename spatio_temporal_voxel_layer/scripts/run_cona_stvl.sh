#!/bin/bash
# Standalone STVL costmap on the CoNA rig — no nav2 stack, just the layer.
#
# Three things here are not obvious and each one produced a silently-broken run before:
#
#  * CYCLONEDDS_URI. The robot's DDS config lives in /etc/coga-robotics and without it a
#    node joins a different view of the graph: `ros2 topic list` shows 2 entries and the
#    costmap subscribes to nothing while still activating cleanly.
#  * The node name is fixed. nav2_costmap_2d's standalone main ignores __node/__ns and
#    always comes up as /costmap/costmap, so both the params-file key and the lifecycle
#    target below have to use that name.
#  * It is a lifecycle node. Launching leaves it UNCONFIGURED and silent; without the two
#    transitions it looks like a healthy process that publishes nothing.
set -e
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-217}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ -f /etc/coga-robotics/conf/cyclonedds.xml ] && \
    export CYCLONEDDS_URI=file:///etc/coga-robotics/conf/cyclonedds.xml

NODE=/costmap/costmap
CFG=$(ros2 pkg prefix spatio_temporal_voxel_layer 2>/dev/null)/share/spatio_temporal_voxel_layer/example/cona_stvl_costmap.yaml
[ -f "$CFG" ] || CFG=$HOME/ros2_ws/src/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer/example/cona_stvl_costmap.yaml
LOG=$HOME/stvl_test/stvl.log

echo "config: $CFG"
pkill -f "nav2_costmap_2d --ros-args" 2>/dev/null || true
sleep 1

setsid nohup ros2 run nav2_costmap_2d nav2_costmap_2d \
    --ros-args --params-file "$CFG" > "$LOG" 2>&1 &
echo "launched pid=$!   log=$LOG"

for i in $(seq 40); do
    ros2 lifecycle nodes 2>/dev/null | grep -qx "$NODE" && break
    sleep 0.5
done

echo "-- configure"; ros2 lifecycle set "$NODE" configure
echo "-- activate";  ros2 lifecycle set "$NODE" activate
echo "-- state: $(ros2 lifecycle get $NODE 2>/dev/null)"
echo
echo "costmap topic : /costmap/costmap   (full grid, always_send_full_costmap)"
echo "stop          : pkill -f 'nav2_costmap_2d --ros-args'"
