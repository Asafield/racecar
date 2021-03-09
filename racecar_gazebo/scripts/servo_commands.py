#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
# from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0


def limsteer(data,maxdata):
    if data>0 and data > maxdata:
        data = maxdata
    elif data<0 and math.fabs(data) > maxdata:
        data = maxdata
    return data
def set_speed(data):
    global flag_move
    
    pub_vel_left_rear_wheel = rospy.Publisher('/servo/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    x = data.linear.x 
    z = data.angular.z
    z= (0 if abs(z) < 0.05 else z)    
    if z == 0:
        x=x*2.28
    elif math.fabs(z)<0.25:
        x=x*1.0
        z=z*1.2
    else:
        x=x*0.92
        z=z*1.76
    vel.linear.x = x 
    vel.angular.z = z        
    pub_vel_left_rear_wheel.publish(vel)
    

def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
    #rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.Subscriber("/vesc/cmd_vel", Twist, set_speed)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass