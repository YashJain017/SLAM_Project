#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class PathControl:

    def __init__(self):
        # Create a ROS publisher
        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.Inittime = rospy.Time.now().to_sec()

    def publish_cmd_vel(self, event=None):
        vel_msg = Twist()

        currentTime = rospy.Time.now().to_sec()
#PATH 1
        if (currentTime-self.Inittime) < 1.9*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (1.9+1.5)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = -0.44
        elif (currentTime-self.Inittime) < (1.9+1.5+9.19)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (1.9+1.5+1.5+9.19)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = -0.44
        elif (currentTime-self.Inittime) < (1.9+1.5+9.19+1.5+5.5)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (1.9+1.5+1.5+9.19+1.5+5.5)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = -0.5
        elif (currentTime-self.Inittime) < (1.9+1.5+9.19+1.5+9.19+1.5+5.5)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        
        self.velocity_pub.publish(vel_msg)
#PATH 2
        """ if (currentTime-self.Inittime) < 1.9*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (1.9+1.5)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.55
        elif (currentTime-self.Inittime) < (1.9+1.5+2.25)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (1.9+1.5+1.5+2.25)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = -0.45
        elif (currentTime-self.Inittime) < (1.9+1.5+1.5+2.25+3)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0 """

#PATH 3        
"""         if (currentTime-self.Inittime) < (6-1.9)*2:
            vel_msg.linear.x = -0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        elif (currentTime-self.Inittime) < (6-1.9+1.5)*2:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.51
        elif (currentTime-self.Inittime) < (6-1.9+1.5+8.5)*2:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0 """
        
        

if __name__ == '__main__':

    rospy.init_node("plan_trajectory")

    # Create an instance of PathControl class
    pc = PathControl()

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(1.0/100.0), pc.publish_cmd_vel)

    # Don't forget this or else the program will exit
    rospy.spin()