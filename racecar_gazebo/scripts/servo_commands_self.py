#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0


def limsteer(data,maxdata):
    if data>0 and data > maxdata:
        data = maxdata
    elif data<0 and math.fabs(data) > maxdata:
        data = maxdata
    return data
def set_speed(data):
    global flag_move
    
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    
    x = data.linear.x  
    z = data.angular.z
    
        vR_front =vR_front*120

    pub_vel_left_rear_wheel.publish(vL_rear)
    

def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
    #rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.Subscriber("/cmd_vel", Twist, set_speed)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass