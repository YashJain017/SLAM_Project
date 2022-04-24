#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class vote:
    def __init__(self):
        self.ekf_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_callback)
        self.hmm_sub = rospy.Subscriber("/markov_chain_pose", PoseStamped, self.hmm_callback)
        self.vote_pub = rospy.Publisher("/vote_pose", PoseStamped, queue_size=10)
        self.vot_pub_timer = rospy.Timer(rospy.Duration(1/100.0), self.soft_vote)
        self.ctr = 0
        self.prev = PoseStamped()
        self.prev.pose.position.x = 0
        self.prev.pose.position.y = 0
        self.prev.pose.orientation.w = 0
        self.ekf = PoseStamped()
        self.hmm = PoseStamped()

    def ekf_callback(self, ekf_msg):
        rospy.loginfo("In EKF Callback")
        self.ekf.pose.position.x = ekf_msg.pose.pose.position.x
        self.ekf.pose.position.y = ekf_msg.pose.pose.position.y
        self.ekf.pose.orientation.w = ekf_msg.pose.pose.orientation.w
        self.ctr += 1

    def hmm_callback(self, hmm_msg):
        rospy.loginfo("In HMM Callback")
        self.hmm.pose.position.x = hmm_msg.pose.position.x
        self.hmm.pose.position.y = hmm_msg.pose.position.y
        self.hmm.pose.orientation.w = hmm_msg.pose.orientation.w

    def soft_vote(self, vot_pub_timer):
        if self.ctr <= 10:
            self.vote_pub.publish(self.ekf)
            self.prev = self.ekf
        else:
            hmm_error = math.sqrt((self.prev.pose.position.x - self.hmm.pose.position.x)**2 + (self.prev.pose.position.y - self.hmm.pose.position.y)**2 + (self.prev.pose.orientation.w - self.hmm.pose.orientation.w)**2)
            ekf_error = math.sqrt((self.prev.pose.position.x - self.ekf.pose.position.x)**2 + (self.prev.pose.position.y - self.ekf.pose.position.y)**2 + (self.prev.pose.orientation.w - self.ekf.pose.orientation.w)**2)
            if hmm_error <= ekf_error:
                self.vote_pub.publish(self.hmm)
                self.prev = self.hmm
            else:
                self.vote_pub.publish(self.ekf)
                self.prev = self.ekf
            rospy.loginfo("[SoftVoting]: Choose HMM")
        
if __name__ == "__main__":
    rospy.init_node('softvote', anonymous=True)
    vote()
    rospy.spin()




