进入 inactive 状态
ros2 service call /my_lifecycle_node/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: ''}}"
进入 active 状态
ros2 service call /my_lifecycle_node/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: ''}}"

ros2 service list | grep change_state
ros2 lifecycle get my_lifecycle_node


