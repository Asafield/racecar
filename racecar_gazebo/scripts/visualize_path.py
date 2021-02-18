#!/usr/bin/env python
# -*- coding: utf8 -*-


import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from costmap_converter.msg import ObstacleArrayMsg
from geometry_msgs.msg import PolygonStamped, Point32, Twist,Pose
from gazebo_msgs.msg import LinkStates
from control_msgs.msg import JointControllerState
from matplotlib.pyplot import MultipleLocator
import numpy as np
import matplotlib.pyplot as plotter
from std_msgs.msg import Header
import tf
import roslib



class PoseNode:
    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.pose_x = []
        self.pose_y = []
        self.pose_w= []
        self.pose_z = []
        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            # ---------> base_link
            arrayIndex = msg.name.index('racecar::base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]

           
            # if len(self.pose_x)<2:
            #   self.pose_y .append(self.last_received_pose.position.x)
            #   self.pose_y .append(self.last_received_pose.position.y)
            # elif abs(self.last_received_pose.position.x - self.pose_y[-1])>0.01:
            #   self.pose_y .append(self.last_received_pose.position.x)
            #   self.pose_y .append(self.last_received_pose.position.y)



  
def velocity_plotter():
  global node
  rospy.init_node("visualize_obstacle_velocity_profile", anonymous=True)
  # plotter.ion()
  # plotter.show()
  listener = tf.TransformListener()
  
  plotter.figure(figsize=(15,4))
  node.pose_x.append(node.last_received_pose.position.x)
  node.pose_y.append(node.last_received_pose.position.y)
  while(node.pose_x[0] == 0):
    node.pose_x[0] = node.last_received_pose.position.x
    node.pose_y[0] = node.last_received_pose.position.y
  while not rospy.is_shutdown():
    try:
      (trans,rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    plotter.cla()
    

    node.pose_x .append(round(node.last_received_pose.position.x,3))
    node.pose_y .append(round(node.last_received_pose.position.y,3))
    node.pose_w.append(round(trans[0],3))
    node.pose_z.append(round(trans[1],3))

    plotter.plot(node.pose_x,node.pose_y,'-bx',label="true value")     
    plotter.plot(node.pose_w,node.pose_z,'-rx',label="estimated value")  
    # x_major_locator=MultipleLocator(1) 
    # y_major_locator=MultipleLocator(1)
    # ax=plotter.gca()
    # ax.xaxis.set_major_locator(x_major_locator)
    # ax.yaxis.set_major_locator(y_major_locator)
    plotter.legend(loc='upper left') 
    plotter.axis('equal')
    plotter.xlim((-5,15))
    plotter.ylim((-6,4))
    plotter.title('compare_figure')
    # fig.canvas.draw()
    plotter.pause(0.02)
    
        
        

if __name__ == '__main__': 
  try:
   
    node = PoseNode()
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass
  finally:
    plotter.savefig('/home/asafield/visulize_path.pdf', bbox_inches='tight')

