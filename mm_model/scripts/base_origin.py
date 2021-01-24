#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *
import math
from gazebo_msgs.msg import ModelState

rospy.init_node('init_node')
cmd_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'mm'
robot_state = get_state_service(model)
set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
model = SetModelStateRequest()

desired_position=[0,0,0]

desired_state=ModelState()
desired_state.model_name='mm'
desired_state.pose=robot_state.pose
desired_state.pose.position.x = desired_position[0]
desired_state.pose.position.y = desired_position[1]
desired_state.pose.orientation.z = desired_position[2]
set_state_service(desired_state)
rate=rospy.Rate(10)
count=0
while count < 5:
    cmd=Twist()
    cmd_pub.publish(cmd)
    rate.sleep()
    count+=1
model = GetModelStateRequest()
model.model_name = 'mm'
robot_state = get_state_service(model)
print('position_x: '+str(robot_state.pose.position.x))
print('position_y: '+str(robot_state.pose.position.y))
print('orientation_z: '+str(robot_state.pose.orientation.z))