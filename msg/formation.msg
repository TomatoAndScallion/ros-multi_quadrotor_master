std_msgs/Header header
# 编队节点总数
uint8 formation_num 
#本节点要发给的节点编号
uint8 uav_num
#相对位置和相对yaw角。
geometry_msgs/Pose[] ralative_pose
float64[] ralative_yaw

