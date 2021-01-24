#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import Twist



def main():
    rospy.init_node('base_control')
    vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)

    time=[]
    base_x=[]
    base_y=[]

    with open('../data/base_path.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            time.append(data[0])
            base_x.append(data[1])
            base_y.append(data[2])

    rate=rospy.Rate(100)
    count=0
    index=0
    rospy.loginfo('starting to control the mobile base')
    while not rospy.is_shutdown():
        vel=Twist()
        if index < len(base_x)-1:
            vel.linear.x=(base_x[index+1]-base_x[index])/0.1
            vel.linear.y=(base_y[index+1]-base_y[index])/0.1
        elif count == len(base_x)*10:
            rospy.loginfo('the base command has stopped')
        vel_pub.publish(vel)
        count+=1
        index=count/10
        rate.sleep()

if __name__ == '__main__':
    main()