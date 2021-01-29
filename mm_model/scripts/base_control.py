#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import Twist
import mm_model.cubic_spline as sp
from std_msgs.msg import Bool
from gazebo_msgs.msg import LinkStates,ModelStates

class base_class:
    def __init__(self,dir):
        time=[]
        base_x=[]
        base_y=[]

        with open(dir,'r') as f:
            for line in f.readlines():
                data=list(map(float,line.split()))
                time.append(data[0]/10)
                base_x.append(data[1]/1000)
                base_y.append(data[2]/1000)
        self.x_spline=sp.cubic_spline(time,base_x,option=1)
        self.y_spline=sp.cubic_spline(time,base_y,option=1)
        self.start_base=False
        self.max_time=time[-1]

        self.current_x=0
        self.current_y=0
        self.init_x=base_x[0]
        self.init_y=base_y[0]


    def callback(self,msg):
        self.start_base=msg.data
    
    def state_callback(self,msg):
        if self.start_base:
            with open('../data/actual_ee_path.txt','a') as f:
                #the distance between the manipulator and centern of the base 
                f.write(str(msg.pose[-1].position.x-0.3)+' '+str(msg.pose[-1].position.y)+'\r\n')
    
    def base_callback(self,msg):
        self.current_x=msg.pose[-1].position.x
        self.current_y=msg.pose[-1].position.y
        if self.start_base:
            with open('../data/actual_base_path.txt','a') as f:
                #the distance between the manipulator and centern of the base 
                f.write(str(self.current_x)+' '+str(self.current_y)+'\r\n')
    
    def get_delta_vel(self,time):
        delta_vel_x=(self.x_spline.get_position(time)-self.current_x)*0.8
        delta_vel_y=(self.y_spline.get_position(time)-self.current_y)*0.8

        #print(str(self.y_spline.get_position(time))+' '+str(self.current_y))
        return delta_vel_x,delta_vel_y

    def goto_init_pos(self,vel_pub,rate):
        vel=Twist()
        while (abs(self.init_x-self.current_x)>0.01 or abs(self.init_y-self.current_y)>0.01):
            if self.init_x> self.current_x+0.01:
                vel.linear.x=0.1
            elif self.init_x< self.current_x-0.01:
                vel.linear.x=-0.1
            if self.init_y> self.current_y+0.01:
                vel.linear.y=0.1
            elif self.init_y< self.current_y-0.01:
                vel.linear.y=-0.1

            vel_pub.publish(vel)
            rate.sleep()

        return



def main():
    rospy.init_node('base_control')

    dir='../data/base_path.txt'
    base=base_class(dir)

    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    start_sub=rospy.Subscriber('base_start',Bool,base.callback)
    states_sub=rospy.Subscriber('gazebo/link_states',LinkStates,base.state_callback)
    base_sub=rospy.Subscriber('gazebo/model_states',ModelStates,base.base_callback)

    with open('../data/actual_ee_path.txt','w') as f:
        f.write(' ')
    with open('../data/actual_base_path.txt','w') as f:
        f.write(' ')

    hz=100
    rate=rospy.Rate(hz)
    count=0
    init_time=rospy.get_time()
    while init_time==0:
        init_time=rospy.get_time()

    rospy.loginfo('please press enter to go to the initial pose')
    raw_input()
    base.goto_init_pos(vel_pub,rate)

    rospy.loginfo('base initialized, waiting fot the initialization of the manipulator')

    while not rospy.is_shutdown():
        vel=Twist()
        if base.start_base:
            if count==0:
                init_time=rospy.get_time()
                count+=1
                rospy.loginfo('start controlling the mobile base')
                
            time=rospy.get_time()-init_time
            
            if time+2/hz > base.max_time:
                rospy.loginfo('base mission compeleted!')
                base.start_base=False
            else:

                delta_vel_x,delta_vel_y=base.get_delta_vel(time)
                #print(str(delta_vel_x)+' '+str(delta_vel_y))
                if base.x_spline.get_first_derivative(time)>0:
                    vel.linear.x=min(0.5,base.x_spline.get_first_derivative(time))
                else:
                    vel.linear.x=max(-0.5,base.x_spline.get_first_derivative(time))

                if base.y_spline.get_first_derivative(time)>0:
                    vel.linear.y=min(0.5,base.y_spline.get_first_derivative(time))
                else:
                    vel.linear.y=max(-0.5,base.y_spline.get_first_derivative(time))

                vel.linear.x+=delta_vel_x
                vel.linear.y+=delta_vel_y

        else:
            count=0
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':

    main()