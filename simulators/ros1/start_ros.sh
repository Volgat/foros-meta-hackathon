#!/bin/bash
# Start ROS1 core and Gazebo simulation
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export DISPLAY=:1

echo "[FOROS ROS1] Starting roscore..."
roscore &
sleep 4

echo "[FOROS ROS1] Starting rosbridge websocket server on port 9090..."
roslaunch --wait rosbridge_server rosbridge_websocket.launch port:=9090 &
sleep 2

echo "[FOROS ROS1] Starting joint state publisher..."
# Generic 6-DOF joint state simulation
python3 - << 'PYEOF'
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math, time, json, os

rospy.init_node('foros_sim_publisher', anonymous=True)
pub_joints = rospy.Publisher('/joint_states', JointState, queue_size=10)
pub_tcp    = rospy.Publisher('/foros/tcp_pose', PoseStamped, queue_size=10)

joints = [0.0] * 6
rate = rospy.Rate(10)  # 10 Hz feedback

print("[FOROS ROS1] Simulation publisher running...")
while not rospy.is_shutdown():
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    msg.position = joints
    msg.velocity = [0.0] * 6
    msg.effort   = [0.0] * 6
    pub_joints.publish(msg)
    rate.sleep()
PYEOF
