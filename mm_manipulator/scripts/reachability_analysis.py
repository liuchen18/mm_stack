#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import RobotState, RobotTrajectory
import numpy as np
import numpy.matlib
from geometry_msgs.msg import Pose,Quaternion
import math
import copy
import random
import matplotlib.pyplot as plt



class IIWA():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self.group.allow_replanning=True
        self.group.set_pose_reference_frame('iiwa_link_0')
        self.group.set_end_effector_link('iiwa_link_7')
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_max_velocity_scaling_factor(1)
    '''
    def compute_inverse_kinematics(self,end_effector_pose):
        
        #compute the inverse kinematics of the given end effector pose,return joint values,end_effector_pose should be a pose
        
        request=GetPositionIKRequest()
        request.ik_request.group_name=self.group_name
        request.ik_request.ik_link_name = "pan_link"
        request.ik_request.attempts = 100
        request.ik_request.pose_stamped.header.frame_id = "wx"
        request.ik_request.pose_stamped.pose.position.x = end_effector_pose.position.x
        request.ik_request.pose_stamped.pose.position.y = end_effector_pose.position.y
        request.ik_request.pose_stamped.pose.position.z = end_effector_pose.position.z
        request.ik_request.pose_stamped.pose.orientation.x = end_effector_pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = end_effector_pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = end_effector_pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = end_effector_pose.orientation.w
        ik_response=self.compute_ik(request)
        #print(ik_response)
        joint_value=ik_response.solution.joint_state.position
        joint_values=[]
        if len(joint_value) < 10:
            rospy.logerr('the given end_effector_pose has no results')
            return joint_values
        else:
            for i in range(len(joint_value)):
                joint_values.append(joint_value[i])
            #print(joint_value)
            return joint_values
    '''
    def compute_forward_kinematics(self,joint_values,goal_link):
        '''
        compute the forward kinematics of the given joint values with reference to the reference link, return a posestamped
        '''
        fk_request=GetPositionFKRequest()
        links=self.robot.get_link_names()
        fk_request.fk_link_names=links
        state=RobotState()
        joint_names=['iiwa_joint_1','iiwa_joint_2','iiwa_joint_3','iiwa_joint_4','iiwa_joint_5','iiwa_joint_6','iiwa_joint_7']
        state.joint_state.name=joint_names
        state.joint_state.position=joint_values
        fk_request.robot_state=state
        fk_response=self.compute_fk(fk_request)
        index=fk_response.fk_link_names.index(goal_link)
        end_effector_pose=fk_response.pose_stamped[index]
        return end_effector_pose

    
    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        if len(joint_value) < 7:
            r=[]
            return []
        else:
            jacobian_matrix_m=numpy.matlib.zeros((6,6))
            jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
            jacobian_matrix=np.asarray(jacobian_matrix_m)
            return jacobian_matrix

    def compute_cartisian_trajectory(self,way_points):
        '''compute the trajectory according to the compute cartisian path api.inputs are waypoints
        '''
        trajectory=RobotTrajectory()
        attempts=0
        max_attempts=1000
        fraction=0
        while fraction<1 and attempts<max_attempts:
            trajectory,fraction=self.group.compute_cartesian_path(way_points,0.01,0)
            attempts+=1
            if attempts % 20 == 0:
                rospy.loginfo('still trying to solve the path after %d appempts',attempts)
        if fraction == 1:
            rospy.loginfo('the cartisian trajectory computed')
            return trajectory
        else:
            rospy.logerr('cannot solve the path 100%,only' +str(fraction*100)+'% planned')
            exit()

    def compute_manipulability(self,joint_values):
        '''
        compute the manipulability of the given end_effector_pose,return manipulability
        '''
        jacobian_matrix=self.compute_jacobian_matrix(joint_values)
        if jacobian_matrix != []:
            manipulability=math.sqrt(np.linalg.det(np.dot(jacobian_matrix,np.transpose(jacobian_matrix))))
        else:
            manipulability=0
        return manipulability


def main():
    robot=IIWA()
    joint_values=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal_link='iiwa_link_ee'
    maxi=0.0
    resolution=10
    tl=2000
    th=1320
    valid_state_num=0
    positive_num=0
    
    with open('reachability_t.txt','w') as f:
        f.write(' ')
    
    num=tl/resolution
    num_h=th/resolution
    #do not care about the z <0
    reachability=[[[0.0 for i in range(num_h+1)] for j in range(num+1)] for k in range(num+1)]
    iteration=10000000
    for i in range(iteration):
        joint_values[0]=random.uniform(-170*math.pi/180,170*math.pi/180)
        joint_values[1]=random.uniform(-120*math.pi/180,120*math.pi/180)
        joint_values[2]=random.uniform(-170*math.pi/180,170*math.pi/180)
        joint_values[3]=random.uniform(-120*math.pi/180,120*math.pi/180)
        joint_values[4]=random.uniform(-170*math.pi/180,170*math.pi/180)
        joint_values[5]=random.uniform(-120*math.pi/180,120*math.pi/180)
        joint_values[6]=random.uniform(-175*math.pi/180,175*math.pi/180)

        
        #compute forward kinematics and manipulability
        pose=robot.compute_forward_kinematics(joint_values,goal_link).pose

        if pose.position.z<0:
            continue
        positive_num+=1

        mani=robot.compute_manipulability(joint_values)
        maxi=max(maxi,mani)

        #record the max manipulability
        if reachability[int(pose.position.x*1000/resolution)+num/2][int(pose.position.y*1000/resolution)+num/2][int(pose.position.z*1000/resolution)]==0.0:
            valid_state_num+=1

        current=max(reachability[int(pose.position.x*1000/resolution)+num/2][int(pose.position.y*1000/resolution)+num/2][int(pose.position.z*1000/resolution)],mani)
        reachability[int(pose.position.x*1000/resolution)+num/2][int(pose.position.y*1000/resolution)+num/2][int(pose.position.z*1000/resolution)]=current
        
        #print(pose)
        #print(current)

        if i%1000==0:
            print(str(i*1.0/iteration *100)+'% finished!')
        
    
    print('start writing the result to txt file')
    for i in range(num):
        for j in range(num):
            for k in range(num_h):
                with open('reachability_t.txt','a') as f:
                    f.write(str((i-num/2)*resolution)+' '+str((j-num/2)*resolution)+' '+str(k*resolution)+' '+str(reachability[i][j][k])+'\r\n')
    

    print('the max manipulability is: '+str(maxi))
    print('valid state num is : '+str(valid_state_num))
    print('positive state num is : '+str(positive_num))

    


if __name__ == '__main__':
    main()



