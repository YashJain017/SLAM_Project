#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <utility>
#include <algorithm>
#include <bits/stdc++.h>
#include <random>
#include <numeric>


class NoisyData {

    private:
        ros::NodeHandle private_nh;
        ros::NodeHandle *nh_;
        // declare your publishers
        ros::Publisher Imupub;
        ros::Publisher Odompub;

        // declare your subscribers
        ros::Subscriber Imu_sub;
        ros::Subscriber Odom_sub;
    
    public:
        NoisyData(ros::NodeHandle *nh):nh_(nh) ,private_nh("~") {

            
            // initialize your publishers
            Imupub = nh_->advertise<sensor_msgs::Imu>("/noisy_imu", 1000);
            Odompub = nh_->advertise<nav_msgs::Odometry>("/noisy_odom", 1000);

            //initialize your subscribers
            Imu_sub  = nh_->subscribe("mobile_base/sensors/imu_data", 1000, &NoisyData::ImuCallback, this);
            Odom_sub = nh_->subscribe("/odom", 1000, &NoisyData::OdomCallback, this);

        }

        double GaussianNoise(double mean){
            double sigma = 0.01;       
            std::random_device rd{};
            std::mt19937 gen{rd()};
            std::normal_distribution<> d{mean,sigma};

            double noise = d(gen);

            return noise;

        }
        
        void ImuCallback(const sensor_msgs::Imu& imuMsg) {

            sensor_msgs::Imu noisyIMUData;
            noisyIMUData.header = imuMsg.header;
            noisyIMUData.orientation.x = GaussianNoise(imuMsg.orientation.x);
            noisyIMUData.orientation.y = GaussianNoise(imuMsg.orientation.y);
            noisyIMUData.orientation.z = GaussianNoise(imuMsg.orientation.z);
            noisyIMUData.orientation.x = GaussianNoise(imuMsg.orientation.w);
            noisyIMUData.angular_velocity.x = GaussianNoise(imuMsg.angular_velocity.x);
            noisyIMUData.angular_velocity.y = GaussianNoise(imuMsg.angular_velocity.y);
            noisyIMUData.angular_velocity.z = GaussianNoise(imuMsg.angular_velocity.z);
            noisyIMUData.linear_acceleration.x = GaussianNoise(imuMsg.linear_acceleration.x);
            noisyIMUData.linear_acceleration.y = GaussianNoise(imuMsg.linear_acceleration.y);
            noisyIMUData.linear_acceleration.z = GaussianNoise(imuMsg.linear_acceleration.z);
            Imupub.publish(noisyIMUData);
        }

        void OdomCallback(const nav_msgs::Odometry& odomMsg) {

            nav_msgs::Odometry noisyOdomData;
            noisyOdomData.header = odomMsg.header;
            noisyOdomData.child_frame_id = odomMsg.child_frame_id;
            noisyOdomData.pose.pose.position.x = GaussianNoise(odomMsg.pose.pose.position.x);
            noisyOdomData.pose.pose.position.y = GaussianNoise(odomMsg.pose.pose.position.y);
            noisyOdomData.pose.pose.position.z = GaussianNoise(odomMsg.pose.pose.position.z);
            noisyOdomData.pose.pose.orientation.x = GaussianNoise(odomMsg.pose.pose.orientation.x);
            noisyOdomData.pose.pose.orientation.y = GaussianNoise(odomMsg.pose.pose.orientation.y);
            noisyOdomData.pose.pose.orientation.z = GaussianNoise(odomMsg.pose.pose.orientation.z);
            noisyOdomData.pose.pose.orientation.w = GaussianNoise(odomMsg.pose.pose.orientation.w);
            noisyOdomData.twist.twist.linear.x = GaussianNoise(odomMsg.twist.twist.linear.x);
            noisyOdomData.twist.twist.linear.y = GaussianNoise(odomMsg.twist.twist.linear.y);
            noisyOdomData.twist.twist.linear.z = GaussianNoise(odomMsg.twist.twist.linear.z);
            noisyOdomData.twist.twist.angular.x = GaussianNoise(odomMsg.twist.twist.angular.x);
            noisyOdomData.twist.twist.angular.y = GaussianNoise(odomMsg.twist.twist.angular.y);
            noisyOdomData.twist.twist.angular.z = GaussianNoise(odomMsg.twist.twist.angular.z);
            Odompub.publish(noisyOdomData);
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "noisy_ekf");
    ros::NodeHandle nh;
    NoisyData nd = NoisyData(&nh);
    ros::spin();
    
}