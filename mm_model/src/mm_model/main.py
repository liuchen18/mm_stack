#!/usr/bin/env python  

import rospy
import sys
from Executor import Executor
#from threading import Thread
#from gazebo_msgs.srv import *
#from gazebo_msgs.msg import ModelState
import math
#from geometry_msgs.msg import Pose

def main():

    # start a new node
    rospy.init_node("executor")

    rate = rospy.Rate(1.0)

    exe = Executor(file_path='/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory_he.txt')

    # listen to initial offset until success
    while not exe.listen_to_initial_offset():
        rospy.logerr("Waiting for listening to initial offset")
        rate.sleep()
    
    rospy.loginfo("Have successfully listened to initial offset")

    # read the joint trajectory from txt file
    exe.read_trajectory()

    # initiate the pose with the first joint trajectory point
    exe.init_pose()
    

    rospy.loginfo('please press enter to send to trajectory goal')
    raw_input()
    rospy.loginfo('sending joint action goal')
    # go to target pose
    # exe.go_to_target_pose([0.315593518306, -0.0583244396675, 0.316494316795, -0.0258618760329, 0.313625004658, 0.104796584025, 0.310698981177])

    # go to the last pose
    # exe.go_to_last_pose()

    # go to other joint trajectory points one by one
    #exe.send_trajectory_one_by_one(0.01)

    # go to other joint trajectory points
    exe.send_trajectory()





if __name__ == "__main__":
    main()