#!/usr/bin/env python

import rospy
import tf2_ros

# To store the pose of the car, including x, y, theta
from geometry_msgs.msg import Pose2D

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from geometry_msgs.msg import Twist

class Executor:

    def __init__(self, file_path='/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt', action_ns='/iiwa/iiwa_controller/follow_joint_trajectory',vel_publish_rate=10):
        """The construtor function of Executor class

        Keyword Arguments:
            file_path {str} -- the path of the joint trajectory file, if in this package, relative path is better (default: {'../data/fwx_planned_trajectory.txt'})
            action_ns {str} -- the action namespace, used for constructing the action client (default: {'/iiwa/iiwa_controller/follow_joint_trajectory'})
        """
        
        # tf series for listening
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        
        # file path
        self._file_path = file_path

        # number of joints
        self._joint_num = 7

        #cmd_vel publish rate
        self.vel_publish_rate=vel_publish_rate

        # a dictionary containing all the values of the joints in time series
        self._joints_pos_dict = dict()
        self._joints_pos_dict['base_x']=list()
        self._joints_pos_dict['x_y']=list()
        self._joints_pos_dict['y_car']=list()
        for idx in range(self._joint_num):
            self._joints_pos_dict['joint_a'+str(idx+1)]=list()

        # a list containing the base's pose in Pose2D
        self._base_pose_list = list()

        # a list containing the time series
        self._time_list = list()

        # since have added x and y offest initially, we need to neutralize this in control
        self._initial_x_offset = 0.0
        self._initial_y_offset = 0.0

        # action client
        self._action_client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)

        # wait for the action server until finally detecting
        rospy.loginfo("Start waiting for the action server")
        self._action_client.wait_for_server()
        rospy.loginfo("Server detected")

        self.vel_pub=rospy.Publisher('/base/cmd_vel',Twist,queue_size=10)

    def listen_to_initial_offset(self):
        try:
            transform_stamped = self._tfBuffer.lookup_transform('world', 'virtual_base', rospy.Time())   
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from world to virtual_base lookup exception")
            return False
        self._initial_x_offset = transform_stamped.transform.translation.x
        self._initial_y_offset = transform_stamped.transform.translation.y
        
        rospy.loginfo("Initially at ({}, {})".format(self._initial_x_offset, self._initial_y_offset))
        
        return True
    
    def read_trajectory(self):
        """Read time series, corresponding car pose and joint values at each time point
        """
        with open(self._file_path,'r') as f:
            for line in f.readlines()[1:]:
                # convert string to float
                time_slice = map(float,line.split())

                # index 0 is time
                self._time_list.append(time_slice[0])

                # index 1, 2, 3 are x, y, theta
                base_pose = Pose2D()
                base_pose.x = time_slice[1]
                base_pose.y = time_slice[2]
                base_pose.theta = time_slice[3]
                self._base_pose_list.append(base_pose)
                self._joints_pos_dict['base_x'].append(time_slice[1]-self._initial_x_offset)
                self._joints_pos_dict['x_y'].append(time_slice[2]-self._initial_y_offset)
                self._joints_pos_dict['y_car'].append(time_slice[3])

                # index from 4 to 10 are joint_a1 to joint_a7
                for idx in range(self._joint_num):
                    self._joints_pos_dict['joint_a'+str(idx+1)].append(time_slice[idx+4])

    def init_pose(self):
        """Drive the arm to go to the first pose
        """

        rospy.loginfo("FWX Starts initiating the pose")

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        goal.trajectory.joint_names.append('base_x')
        goal.trajectory.joint_names.append('x_y')
        goal.trajectory.joint_names.append('y_car')
        trajPt.positions.append(self._joints_pos_dict['base_x'][0])
        trajPt.positions.append(self._joints_pos_dict['x_y'][0])
        trajPt.positions.append(self._joints_pos_dict['y_car'][0])
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            trajPt.positions.append(self._joints_pos_dict[joint_name][0])
        trajPt.time_from_start = rospy.Duration(secs=10.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)
        #print(goal)

        # wait for the result
        rospy.loginfo("FWX is waiting for finishing initial pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        #rospy.loginfo(self._action_client.get_result())

    def send_trajectory(self):
        """Send a goal which contains all the trajectory points to the action server
        """

        rospy.loginfo("Start going to all trajectory points")

        # a goal to be sent to action server
        # this goal will contain all the joint trajectory points read from the txt file
        goal = FollowJointTrajectoryGoal()

        # add joint name
        goal.trajectory.joint_names.append('base_x')
        goal.trajectory.joint_names.append('x_y')
        goal.trajectory.joint_names.append('y_car')
        for idx in range(self._joint_num): 
            goal.trajectory.joint_names.append("joint_a"+str(idx+1))

        # since the first pt has been realized, iteration will start from the second pt 
        for traj_idx in range(len(self._time_list)-1):
            # a joint point in the trajectory
            trajPt = JointTrajectoryPoint()
            trajPt.positions.append(self._joints_pos_dict['base_x'][traj_idx+1])
            trajPt.positions.append(self._joints_pos_dict['x_y'][traj_idx+1])
            trajPt.positions.append(self._joints_pos_dict['y_car'][traj_idx+1])
            for idx in range(self._joint_num):
                joint_name = "joint_a"+str(idx+1)
                trajPt.positions.append(self._joints_pos_dict[joint_name][traj_idx+1])
                #trajPt.velocities.append(0.0)
            # change the float time to duration
            sec=math.floor(self._time_list[traj_idx+1])
            nsec=(self._time_list[traj_idx+1]-sec)*10**9
            trajPt.time_from_start = rospy.Duration(secs=int(sec),nsecs=int(nsec))
            #trajPt.time_from_start = rospy.Duration(secs=self._time_list[traj_idx+1])
            # add the joint trajectory point to the goal
            goal.trajectory.points.append(trajPt)

        #rospy.loginfo("goal has {} points to reach".format(len(goal.trajectory.points)))

        #rospy.loginfo("each of them is ")

        #for traj_idx in range(len(goal.trajectory.points)):
        #    print goal.trajectory.points[traj_idx]

        goal.trajectory.header.stamp = rospy.Time.now()

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for go to other poses")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def go_to_last_pose(self):
        """Drive the arm to go to the last pose
        """
        
        rospy.loginfo("Start going to the last pose")

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        series_length = len(self._time_list)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        goal.trajectory.joint_names.append('base_x')
        goal.trajectory.joint_names.append('x_y')
        goal.trajectory.joint_names.append('y_car')
        trajPt.positions.append(self._joints_pos_dict['base_x'][-1])
        trajPt.positions.append(self._joints_pos_dict['x_y'][-1])
        trajPt.positions.append(self._joints_pos_dict['y_car'][-1])
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            trajPt.positions.append(self._joints_pos_dict[joint_name][-1])
            #trajPt.velocities.append(0.0)
        trajPt.time_from_start = rospy.Duration(secs=3.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for going to the last pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def go_to_target_pose(self, target_joint_values):
        """Drive the arm to go to the pose target pose specified by target_joint_values

        Arguments:
            target_joint_values {list} -- a list of double, each represents the value of the joint position
        """

        rospy.loginfo("Start going to the target pose at {}".format(target_joint_values))

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        goal.trajectory.joint_names.append('base_x')
        goal.trajectory.joint_names.append('x_y')
        goal.trajectory.joint_names.append('y_car')
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            #trajPt.velocities.append(0.0)
        trajPt.positions = target_joint_values
        trajPt.time_from_start = rospy.Duration(secs=3.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for going to the target pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def send_trajectory_one_by_one(self,time_duration):
        """DIFFERENT from `send_trajectory`, send a goal which only contains ONE trajectory point to the action server, iterating for len(self._time_list)-1 times
        """

        rospy.loginfo("Start going to other trajectory points one by one")

        time_last = rospy.Time.now()

        # since the first pt has been realized, iteration will start from the second pt 
        for traj_idx in range(len(self._time_list)-1):
            # a goal to be sent to action server
            # this goal will contain only one trajectory point
            goal = FollowJointTrajectoryGoal()

            # add joint name
            goal.trajectory.joint_names.append('base_x')
            goal.trajectory.joint_names.append('x_y')
            goal.trajectory.joint_names.append('y_car')
            for idx in range(self._joint_num): 
                goal.trajectory.joint_names.append("joint_a"+str(idx+1))

            # a joint point in the trajectory
            trajPt = JointTrajectoryPoint()
            trajPt.positions.append(self._joints_pos_dict['base_x'][idx+1])
            trajPt.positions.append(self._joints_pos_dict['x_y'][idx+1])
            trajPt.positions.append(self._joints_pos_dict['y_car'][idx+1])
            for idx in range(self._joint_num):
                joint_name = "joint_a"+str(idx+1)
                trajPt.positions.append(self._joints_pos_dict[joint_name][traj_idx+1])
                #trajPt.velocities.append(0.0)
            # time to reach the joint trajectory point specified to 1.0 since this will be controlled by my enter
            trajPt.time_from_start = rospy.Duration(secs=time_duration)
            # add the joint trajectory point to the goal
            goal.trajectory.points.append(trajPt)

            # rospy.loginfo("At iteration {} goal has {} points to reach".format(traj_idx ,len(goal.trajectory.points)))

            # rospy.loginfo("each of them is ")

            for idx in range(len(goal.trajectory.points)):
                print goal.trajectory.points[idx]

            goal.trajectory.header.stamp = rospy.Time.now()

            # send the goal to the action server
            self._action_client.send_goal(goal)

            # wait for the result
            rospy.loginfo("Start waiting for go to the next pose")
            self._action_client.wait_for_result()
            rospy.loginfo("Waiting ends")

            # show the error code
            rospy.loginfo(self._action_client.get_result())

            # show the time used to move to current position
            time_next = rospy.Time.now()
            rospy.loginfo("At iteration {}, time Duration is {}".format(traj_idx ,(time_next-time_last).to_sec()))
            time_last = time_next

            if rospy.is_shutdown():
                exit()
            raw_input()
    '''
    def send_base_vel(self):
        rospy.sleep(1.0)#wait fot the arm to move together
        rate=rospy.Rate(self.vel_publish_rate)
        start_time=rospy.get_time()
        current_index=1
        last_index=0
        zero_count=0
        rospy.loginfo('starting to publish velocity command to control the base')
        while zero_count<2:
            last_index=current_index
            #print('len: '+str(len(self._time_list)))
            while current_index < len(self._time_list)-1 and self._time_list[current_index]<rospy.get_time()-start_time+1/self.vel_publish_rate:
                current_index+=1
                #print(current_index)
            if current_index == len(self._time_list)-1:
                vel=Twist()
                self.vel_pub.publish(vel)
                zero_count+=1
            if current_index!=last_index:
                vel=Twist()
                vel.linear.x=(self._base_pose_list[current_index].x-self._base_pose_list[last_index].x)/(self._time_list[current_index]-self._time_list[last_index])
                vel.linear.y=(self._base_pose_list[current_index].y-self._base_pose_list[last_index].y)/(self._time_list[current_index]-self._time_list[last_index])
                vel.angular.z=(self._base_pose_list[current_index].theta-self._base_pose_list[last_index].theta)/(self._time_list[current_index]-self._time_list[last_index])
                self.vel_pub.publish(vel)
            rate.sleep()
        rospy.loginfo('base velocity command over')
    '''