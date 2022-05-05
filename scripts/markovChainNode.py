#!/usr/bin/env python

from turtle import position
import rospy
import json
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
import pickle
import numpy as np
from nav_msgs.msg import Odometry
import copy
import pandas as pd

class MarkovChain:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("markov_chain_pose", PoseStamped, queue_size=10)
        self.number_subscriber = rospy.Subscriber("odom", Odometry, self.pose_callback)
        self.previousPosesBuffer = []
        self.previousPosesBuffer.append((0.0, 0.0, 0.0))
        self.stride = 3
        self.predictedPose = (0.0,0.0,0.0)
        rospy.loginfo("Loading Model")
        with open('/home/rhea/workspace/catkin_ws/src/SLAM_Project/scripts/model_path3_15_combined.pkl','rb') as f:
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
        pose_buf = Odometry()
        PositionX = copy.deepcopy(pose_msg.pose.pose.position.x)
        PositionY = copy.deepcopy(pose_msg.pose.pose.position.y)
        Yaw = copy.deepcopy(pose_msg.pose.pose.orientation.w)
        predictedPose = (0.0, 0.0)
        
        # read pose values and append to buffer
        if ((abs(self.previousPosesBuffer[-1][0]- PositionX) >= 0.1) or (abs(self.previousPosesBuffer[-1][1]- PositionY) >= 0.1) or (abs(self.previousPosesBuffer[-1][2]- Yaw) >= 0.1)):
            df=pd.read_csv(r"/home/rhea/workspace/catkin_ws/src/SLAM_Project/scripts/path3_15combined.csv")
            df1=df.copy()
            position2compare=[PositionX, PositionY, Yaw]
            df1=((df1-position2compare)**2).sum(axis=1).idxmin()
            positionclosest=df.loc[df1]
            PositionX=positionclosest[0]
            PositionY=positionclosest[1]
            Yaw=positionclosest[2]
            self.previousPosesBuffer.append((round(PositionX, 1),round(PositionY,1), round(Yaw,1)))

        
        # call the sample_next function
        self.predictedPose = self.sample_next(self.previousPosesBuffer[-self.stride:], self.model, self.stride)

        # pop the pose from the predicted list if it exceeds a specific length
        if len(self.previousPosesBuffer) > 10:
            self.previousPosesBuffer.pop(0)

        
        # publish the pose message
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = pose_msg.header.frame_id
        new_pose.header.seq   = pose_msg.header.seq
        new_pose.pose.position.x = self.predictedPose[0]
        new_pose.pose.position.y = self.predictedPose[1]
        new_pose.pose.position.z = 0.0
        new_pose.pose.orientation.x = 0.0
        new_pose.pose.orientation.y = 0.0
        new_pose.pose.orientation.z = 0.0
        new_pose.pose.orientation.w = self.predictedPose[2]
        self.pub.publish(new_pose)
    
    def sample_next(self, ctx, model, k):
        ctx = ctx[-k:]
        ctx = str(ctx).strip('[]')
     
        if model.get(ctx) is None:
            return (self.predictedPose[0], self.predictedPose[1], self.predictedPose[2])

        
        possible_Chars = list(model[ctx].keys())
        possible_values = list(model[ctx].values())
        possible_chars_idx = []
        array_possible_poses = np.array(possible_Chars)
        
        for i in range (len(array_possible_poses)):
            possible_chars_idx.append(i)
        
        array_possible_values = np.array(possible_values)
        idx = np.random.choice(possible_chars_idx, p=array_possible_values)
        self.counter+=1
        print(self.counter)
        return possible_Chars[idx]
    

if __name__ == '__main__':
    rospy.init_node('number_counter')
    MarkovChain()
    rospy.spin()
