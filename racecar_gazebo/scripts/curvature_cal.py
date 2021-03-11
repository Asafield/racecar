#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
import numpy.linalg as LA
from move_base_msgs.msg import MoveBaseActionGoal

#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
class path_following:
    def __init__(self):
        self.point_width = 40 #三个点之间的间隔距离
        self.mid_point = 100  #中间的点离小车的距离
        self.flag = 0         #最后一段路程的判断标志，因为本次迅飞的最后一段路程要求不能撞墙，但障碍却少，适合冲刺，所以要求冲的又快又稳，所以需要判断标志
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

        self.pub_point1 = rospy.Publisher("point1", PointStamped, queue_size = 1)
        self.pub_point2 = rospy.Publisher("point2", PointStamped, queue_size = 1)
        self.pub_point3 = rospy.Publisher("point3", PointStamped, queue_size = 1)

        self.point1 = PointStamped()
        self.point2 = PointStamped()
        self.point3 = PointStamped()

        self.speed_data=Float64()
        self.turn_data=Float64()
        self.min_curvature = Float64(1000.0)
        self.cmd_data = Twist()
        self.goal_point_x = Float64()
        self.goal_point_y = Float64()
        self.odom_point_x = Float64()
        self.odom_point_y = Float64()
        self.odom_yaw = Float64()
    def odom_callback(self,data):
        self.odom_point_x = data.pose.pose.position.x
        self.odom_point_y = data.pose.pose.position.y
        self.odom_yaw = data.twist.twist.angular.z
    def get_goal_callback(self,data):
        self.goal_point_x = data.goal.target_pose.pose.position.x
        self.goal_point_y = data.goal.target_pose.pose.position.y

    def lenth_cal(self,x1,x2,y1,y2):
        x = (x1 - x2)**2
        y = (y1 - y2)**2
        l = np.sqrt(x+y)
        return l

    def radius_cal(self,a,b,c):
        p = (a+b+c)/2.0
        s = np.sqrt(p*(p-a)*(p-b)*(p-c))
        r = (a*b*c)/4.0/s
        return r



    def cmd_vel_callback(self,data):
        stop_flag = Float64(self.lenth_cal(float(self.odom_point_x) , float(self.goal_point_x) , float(self.odom_point_y) , float(self.goal_point_y)))
        data.angular.z= (0 if abs(data.angular.z) < 0.05 else data.angular.z)    
        if data.angular.z == 0:
            data.linear.x =data.linear.x *1.6
        elif math.fabs(data.angular.z)<0.25:
            data.linear.x =data.linear.x *1.0
            data.angular.z=data.angular.z*1.2
        else:
            data.linear.x =data.linear.x *0.92
            data.angular.z =data.angular.z *1.76
        if  stop_flag.data <= 0.3:
            self.cmd_data.linear.x = 0
            self.cmd_data.angular.z = 0
        else:
            self.cmd_data.angular.z=data.angular.z
            if self.flag == 1:
                self.cmd_data.linear.x = data.linear.x*1.1
            elif self.flag == 2:
                self.cmd_data.linear.x = data.linear.x*1.3
            elif self.flag == 3:
                self.cmd_data.linear.x = data.linear.x*1.90
            elif self.min_curvature.data <= 3:
                self.cmd_data.linear.x = data.linear.x*1.03
            else:
                self.cmd_data.linear.x = data.linear.x*1.65      
        self.pub_cmd.publish(self.cmd_data)




    def path_callback(self,data):
        # x = len(data.poses) 
        # print(x)  
        if len(data.poses) <= self.point_width*2:
            index1 = 0
            index2 = (len(data.poses)-1)//2
            index3 = len(data.poses)-1
        elif (len(data.poses) > (self.point_width*2)) and (len(data.poses) <= (self.mid_point+self.point_width)):
            index1 = (len(data.poses)-1)-(self.point_width*2)
            index2 = (len(data.poses)-1)-self.point_width
            index3 = len(data.poses)-1
        elif len(data.poses) > (self.mid_point+self.point_width):
            index1 = self.mid_point-self.point_width
            index2 = self.mid_point
            index3 = self.mid_point+self.point_width
        else:
            index1 = index2 = index3 = 0
        if len(data.poses)<=70:
            self.flag = 1
        elif len(data.poses)<=120:
            self.flag = 2
        elif len(data.poses)<=300:
            self.flag = 3
        else:
            self.flag = 0

        self.point1.header.frame_id = 'map'
        self.point1.point.x = data.poses[index1].pose.position.x
        self.point1.point.y = data.poses[index1].pose.position.y
        self.pub_point1.publish(self.point1)

        self.point2.header.frame_id = 'map'
        self.point2.point.x = data.poses[index2].pose.position.x
        self.point2.point.y = data.poses[index2].pose.position.y
        self.pub_point2.publish(self.point2)

        self.point3.header.frame_id = 'map'
        self.point3.point.x = data.poses[index3].pose.position.x
        self.point3.point.y = data.poses[index3].pose.position.y
        self.pub_point3.publish(self.point3)

        if index1 >= 0:
            self.min_curvature = Float64(1000.0)
            for i in range(0,index1): 
                len1 = self.lenth_cal(data.poses[index1-i].pose.position.x , data.poses[index2-i].pose.position.x , data.poses[index1-i].pose.position.y , data.poses[index2-i].pose.position.y)
                len2 = self.lenth_cal(data.poses[index2-i].pose.position.x , data.poses[index3-i].pose.position.x , data.poses[index2-i].pose.position.y , data.poses[index3-i].pose.position.y)
                len3 = self.lenth_cal(data.poses[index1-i].pose.position.x , data.poses[index3-i].pose.position.x , data.poses[index1-i].pose.position.y , data.poses[index3-i].pose.position.y)
                curvature = Float64(self.radius_cal(len1 , len2 , len3))
                if curvature.data <= self.min_curvature.data:
                    self.min_curvature.data = curvature.data
       
    
if __name__ == '__main__':
    try:
        rospy.init_node('curvature_cal', anonymous = True)
        Node = path_following()
        rospy.Subscriber("/vesc/cmd_vel", 
                         Twist, 
                         Node.cmd_vel_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", 
                         Path, 
                         Node.path_callback)       
        rospy.Subscriber("/move_base/goal", 
                         MoveBaseActionGoal, 
                         Node.get_goal_callback)
        rospy.Subscriber("/odom", 
                         Odometry, 
                         Node.odom_callback) 
        
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
