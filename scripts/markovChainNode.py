#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import pickle
import numpy as np

class MarkovChain:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("markov_chain_pose", PoseStamped, queue_size=10)
        self.number_subscriber = rospy.Subscriber("ensemble_pose", PoseStamped, self.pose_callback)
        self.previousPosesBuffer = []
        self.stride = 3
        rospy.loginfo("Loading Model")
        with open('/home/yash/catkin_ws/src/SLAM_Project/scripts/model.pkl','rb') as f:
            self.model = pickle.load(f)
        rospy.loginfo("finished loading model")

    def pose_callback(self, pose_msg):
        """
        Callback function for subscriber. Subscribint to previous pose 
        and publishing Pose using markov chain based on previous set of poses.
        Args:
            pose_msg (_type_): _description_
        """
        new_pose = PoseStamped()
        predictedPose = (0.0, 0.0)
        rospy.loginfo("In pose callback")
        # read pose values and append to buffer
        self.previousPosesBuffer.append((pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.orientation.w))
        
        # call the sample_next function
        predictedPose = self.sample_next(self.previousPosesBuffer[-self.stride:], self.model, self.stride)
                
        # pop the pose from the predicted list if it exceeds a specific length
        if len(self.previousPosesBuffer) > 10.0:
            self.previousPosesBuffer.pop(0)
        
        # publish the pose message
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = pose_msg.header.frame_id
        new_pose.header.seq   = pose_msg.header.seq
        new_pose.pose.position.x = predictedPose[0]
        new_pose.pose.position.y = predictedPose[1]
        new_pose.pose.position.z = 0.0
        new_pose.pose.orientation.x = 0.0
        new_pose.pose.orientation.y = 0.0
        new_pose.pose.orientation.z = 0.0
        new_pose.pose.orientation.w = predictedPose[2]
                
        self.pub.publish(new_pose)
    
    def sample_next(self, ctx, model, k):
        ctx = ctx[-k:]
        ctx = str(ctx).strip('[]')
        if model.get(ctx) is None:
            return (-1000, -1000, -1000)
        
        possible_Chars = list(model[ctx].keys())
        possible_values = list(model[ctx].values())
        possible_chars_idx = []
        array_possible_poses = np.array(possible_Chars)
        
        for i in range (len(array_possible_poses)):
            possible_chars_idx.append(i)
        
        array_possible_values = np.array(possible_values)
        idx = np.random.choice(possible_chars_idx, p=array_possible_values)
        
        return possible_Chars[idx]
    

if __name__ == '__main__':
    rospy.init_node('number_counter')
    MarkovChain()
    rospy.spin()
