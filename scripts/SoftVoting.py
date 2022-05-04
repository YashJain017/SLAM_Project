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
        self.iter = 1
        self.hmmCount = 0
        self.ekfCount = 0
        self.totalcount = 0
        self.resultPose = Odometry()

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
        
        if self.ctr <= 10:
            rospy.loginfo("Choosing EKF")
            self.resultPose.header.stamp = rospy.Time.now()
            self.resultPose.header.frame_id = "base_link"
            self.resultPose.pose.pose.position.x = self.ekf.pose.pose.position.x
            self.resultPose.pose.pose.position.y = self.ekf.pose.pose.position.y
            self.resultPose.pose.pose.position.z = 0.0
            self.vote_pub.publish(self.resultPose)
            #self.prev = copy.deepcopy(self.ekf)
            self.ekfCount+=1
            #self.vote_pub.publish(self.resultPose)
        else:
            rospy.loginfo("Choosing HMM")
            # hmm_error = math.sqrt((self.prev.pose.position.x - self.hmm.pose.position.x)**2 + (self.prev.pose.position.y - self.hmm.pose.position.y)**2 + (self.prev.pose.orientation.w - self.hmm.pose.orientation.w)**2)
            # ekf_error = math.sqrt((self.prev.pose.position.x - self.ekf.pose.position.x)**2 + (self.prev.pose.position.y - self.ekf.pose.position.y)**2 + (self.prev.pose.orientation.w - self.ekf.pose.orientation.w)**2)
            # print("EKF Error: "+str(ekf_error))
            
            # self.EKFSum = self.EKFSum + ekf_error
            
            # self.EKFAvg = self.EKFSum/self.iter
            # if ekf_error > 0.31:
            # #if hmm_error > ekf_error:
            #     self.vote_pub.publish(self.hmm)
            #     self.prev = copy.deepcopy(self.hmm)
            #     rospy.loginfo("[SoftVoting]: Choose HMM")
            #     self.hmmCount+=1
            # else:
            #     self.vote_pub.publish(self.ekf)
            #     self.prev = copy.deepcopy(self.ekf)
            #     rospy.loginfo("[SoftVoting]: Choose EKF")
            #     self.ekfCount+=1

            # print("EKF Error avg: "+str(self.EKFAvg))
            # self.iter = self.iter+1
        #self.totalcount = self.ekfCount + self.hmmCount
            self.resultPose.header.stamp = rospy.Time.now()
            self.resultPose.header.frame_id = "base_link"
            self.resultPose.pose.pose.position.x = self.ekf.pose.pose.position.x#(self.hmm.pose.position.x + self.ekf.pose.pose.position.x)/2
            self.resultPose.pose.pose.position.y = self.ekf.pose.pose.position.y#(self.hmm.pose.position.y + self.ekf.pose.pose.position.y)/2
            self.resultPose.pose.pose.position.z = 0.0
            self.resultPose.pose.pose.orientation.w=self.ekf.pose.pose.orientation.w
            self.vote_pub.publish(self.resultPose)
        # print("EKF use : "+str(self.ekfCount))
        # print("HMM use : "+str(self.hmmCount))
        self.ctr+=1

                
        
if __name__ == "__main__":
    rospy.init_node('softvote', anonymous=True)
    vote()
    rospy.spin()




