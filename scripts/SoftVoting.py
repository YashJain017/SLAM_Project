#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import copy

class vote:
    def __init__(self):
        self.ekf_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_callback)
        self.hmm_sub = rospy.Subscriber("/markov_chain_pose", PoseStamped, self.hmm_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.vote_pub = rospy.Publisher("/vote_pose", Odometry, queue_size=10)
        self.vot_pub_timer = rospy.Timer(rospy.Duration(1/100.0), self.soft_vote)
        self.ctr = 0
        self.prev = PoseStamped()
        self.prev.pose.position.x = 0
        self.prev.pose.position.y = 0
        self.prev.pose.orientation.w = 0
        self.ekf = PoseWithCovarianceStamped()
        self.hmm = PoseStamped()
        self.toggle = True
        self.EKFSum = 0.0
        self.EKFAvg = 0.0
        self.mse = 0.0
        self.iter = 1
        self.hmmCount = 0
        self.ekfCount = 0
        self.totalcount = 0
        self.resultPose = Odometry()
        self.odom = Odometry()
        self.hmm_error = 0.0
        self.ekf_error = 0.0
        self.ekf_error1 = 0.0
        self.hmm_error1 = 0.0

        
    def odom_callback(self, odom_msg):
        self.odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        self.odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

    def ekf_callback(self, ekf_msg):
        #rospy.loginfo("In EKF Callback")
        self.ekf.pose.pose.position.x = ekf_msg.pose.pose.position.x
        self.ekf.pose.pose.position.y = ekf_msg.pose.pose.position.y
        self.ekf.pose.pose.orientation.w = ekf_msg.pose.pose.orientation.w
        self.ctr += 1

    def hmm_callback(self, hmm_msg):
        #rospy.loginfo("In HMM callback")
        self.hmm.pose.position.x = hmm_msg.pose.position.x
        self.hmm.pose.position.y = hmm_msg.pose.position.y
        self.hmm.pose.orientation.w = hmm_msg.pose.orientation.w

    def soft_vote(self, vot_pub_timer):

        # Threshold and Closest to path ##################################################################################################################################################
        if self.ctr <= 10:
            rospy.loginfo("Choosing EKF")
            self.resultPose.header.stamp = rospy.Time.now()
            self.resultPose.header.frame_id = "base_link"
            self.resultPose.pose.pose.position.x = self.ekf.pose.pose.position.x
            self.resultPose.pose.pose.position.y = self.ekf.pose.pose.position.y
            self.resultPose.pose.pose.position.z = 0.0
            self.resultPose.pose.pose.orientation.w=self.ekf.pose.pose.orientation.w
            self.vote_pub.publish(self.resultPose)
            self.ekfCount+=1 
            error = math.sqrt((self.resultPose.pose.pose.position.x - self.odom.pose.pose.position.x)**2 + (self.resultPose.pose.pose.position.y - self.odom.pose.pose.position.y)**2)
            print("Error for comparison: ", self.ekf_error1)
        else:
            error = math.sqrt((self.resultPose.pose.pose.position.x - self.odom.pose.pose.position.x)**2 + (self.resultPose.pose.pose.position.y - self.odom.pose.pose.position.y)**2)
            print("Error for comparison: ", self.ekf_error1)
            if self.ekf_error1 > self.hmm_error1:  # uncomment for closest path approach
            # if self.ekf_error1 > 0.04: # uncomment for threshold approach
                rospy.loginfo("Choosing HMM")
                self.resultPose.header.stamp = rospy.Time.now()
                self.resultPose.header.frame_id = "base_link"
                self.resultPose.pose.pose.position.x = self.hmm.pose.position.x
                self.resultPose.pose.pose.position.y = self.hmm.pose.position.y
                self.resultPose.pose.pose.position.z = 0.0
                self.resultPose.pose.pose.orientation.w=self.hmm.pose.orientation.w
                self.hmmCount+=1
                self.vote_pub.publish(self.resultPose)
            else:
                rospy.loginfo("Choosing EKF")
                self.resultPose.header.stamp = rospy.Time.now()
                self.resultPose.header.frame_id = "base_link"
                self.resultPose.pose.pose.position.x = self.ekf.pose.pose.position.x#(self.hmm.pose.position.x + self.ekf.pose.pose.position.x)/2
                self.resultPose.pose.pose.position.y = self.ekf.pose.pose.position.y#(self.hmm.pose.position.y + self.ekf.pose.pose.position.y)/2
                self.resultPose.pose.pose.position.z = 0.0
                self.resultPose.pose.pose.orientation.w=self.ekf.pose.pose.orientation.w
                self.vote_pub.publish(self.resultPose)
                self.ekfCount+=1
        ##################################################################################################################################################################################

        # AVERAGING #######################################################################################
        # self.resultPose.pose.pose.position.x = (self.ekf.pose.pose.position.x + self.hmm.pose.position.x)/2
        # self.resultPose.pose.pose.position.y = (self.ekf.pose.pose.position.y + self.hmm.pose.position.y)/2
        # self.resultPose.pose.pose.position.z = 0.0
        # ###################################################################################################

        self.hmm_error1 = math.sqrt((self.odom.pose.pose.position.x - self.hmm.pose.position.x)**2 + (self.odom.pose.pose.position.y - self.hmm.pose.position.y)**2)
        self.ekf_error1 = math.sqrt((self.odom.pose.pose.position.x - self.ekf.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - self.ekf.pose.pose.position.y)**2)

        self.hmm_error += math.sqrt((self.odom.pose.pose.position.x - self.hmm.pose.position.x)**2 + (self.odom.pose.pose.position.y - self.hmm.pose.position.y)**2)
        self.ekf_error += math.sqrt((self.odom.pose.pose.position.x - self.ekf.pose.pose.position.x)**2 + (self.odom.pose.pose.position.y - self.ekf.pose.pose.position.y)**2)
        
        self.vote_pub.publish(self.resultPose)

        print("EKF use : "+str(self.ekfCount))
        print("HMM use : "+str(self.hmmCount))
        error = math.sqrt((self.resultPose.pose.pose.position.x - self.odom.pose.pose.position.x)**2 + (self.resultPose.pose.pose.position.y - self.odom.pose.pose.position.y)**2)
        self.mse += error
        self.ctr+=1
        print('SoftVoting error:',self.mse/self.ctr)
        print("EKF Error: "+str(self.ekf_error/self.ctr))
        print("HMM Error: "+str(self.hmm_error/self.ctr))

if __name__ == "__main__":
    rospy.init_node('softvote', anonymous=True)
    vote()
    rospy.spin()




