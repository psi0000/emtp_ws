#!/bin/bash
BAG_DIR="bag"

mkdir -p ${BAG_DIR}

BAG_NAME="${BAG_DIR}/bag_$(date +"%Y%m%d_%H%M%S")"

ros2 bag record \
  -o ${BAG_NAME} \
  /robot1/map_3d \
  /task_wp \
  /task_2F_wp \
  /task_3F_wp \
  /trigger_markers \
  /fleet_states \
  /task_complete \
  /task_pub \
  /cancel_robot \
  /dynamic_event \
  /robot1/current_target \
  /robot2/current_target \
  /robot3/current_target \
  /allocation \
  /robot1/odom \
  /robot1/localization_path \
  /robot2/odom \
  /robot2/localization_path \
  /robot3/odom \
  /robot3/localization_path \
  /tf \
  /tf_static
