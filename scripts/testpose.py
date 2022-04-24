#!/usr/bin/env python
from email import header
import rospy
from geometry_msgs.msg import PoseStamped


def talker():
    pub = rospy.Publisher('ensemble_pose', PoseStamped, queue_size=10)
    rospy.init_node('test_pose', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    pub_pose = PoseStamped()
    while not rospy.is_shutdown():
        pub_pose.header.seq = i
        pub_pose.header.frame_id = "base"
        pub_pose.pose.position.x = i
        pub_pose.pose.position.y = 0
        pub_pose.pose.position.z = 0
        i += 1
        pub.publish(pub_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass