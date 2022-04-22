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

        double GaussianNoise(double variance = 0, double mean = 0){

            double sigma = sqrt(variance);            
            std::default_random_engine generator;
            std::normal_distribution<double> distribution(mean,sigma);
            double noise = distribution(generator);

            return noise;

        }
        
        void ImuCallback(const sensor_msgs::Imu& imuMsg) {

            sensor_msgs::Imu noisyIMUData;
            noisyIMUData.orientation.x = imuMsg.orientation.x + GaussianNoise();
            noisyIMUData.orientation.y = imuMsg.orientation.y + GaussianNoise();
            noisyIMUData.orientation.z = imuMsg.orientation.z + GaussianNoise();
            noisyIMUData.orientation.x = imuMsg.orientation.w + GaussianNoise();
            noisyIMUData.angular_velocity.x = imuMsg.angular_velocity.x + GaussianNoise();
            noisyIMUData.angular_velocity.y = imuMsg.angular_velocity.y + GaussianNoise();
            noisyIMUData.angular_velocity.z = imuMsg.angular_velocity.z + GaussianNoise();
            noisyIMUData.linear_acceleration.x = imuMsg.linear_acceleration.x + GaussianNoise();
            noisyIMUData.linear_acceleration.y = imuMsg.linear_acceleration.y + GaussianNoise();
            noisyIMUData.linear_acceleration.z = imuMsg.linear_acceleration.z + GaussianNoise();
            Imupub.publish(noisyIMUData);
        }

        void OdomCallback(const nav_msgs::Odometry& odomMsg) {

            nav_msgs::Odometry noisyOdomData;
            noisyOdomData.pose.pose.position.x = odomMsg.pose.pose.position.x + GaussianNoise();
            noisyOdomData.pose.pose.position.y = odomMsg.pose.pose.position.y + GaussianNoise();
            noisyOdomData.pose.pose.position.z = odomMsg.pose.pose.position.z + GaussianNoise();
            noisyOdomData.pose.pose.orientation.x = odomMsg.pose.pose.orientation.x + GaussianNoise();
            noisyOdomData.pose.pose.orientation.y = odomMsg.pose.pose.orientation.y + GaussianNoise();
            noisyOdomData.pose.pose.orientation.z = odomMsg.pose.pose.orientation.z + GaussianNoise();
            noisyOdomData.pose.pose.orientation.w = odomMsg.pose.pose.orientation.w + GaussianNoise();
            noisyOdomData.twist.twist.linear.x = odomMsg.twist.twist.linear.x + GaussianNoise();
            noisyOdomData.twist.twist.linear.y = odomMsg.twist.twist.linear.y + GaussianNoise();
            noisyOdomData.twist.twist.linear.z = odomMsg.twist.twist.linear.z + GaussianNoise();
            noisyOdomData.twist.twist.angular.x = odomMsg.twist.twist.angular.x + GaussianNoise();
            noisyOdomData.twist.twist.angular.y = odomMsg.twist.twist.angular.y + GaussianNoise();
            noisyOdomData.twist.twist.angular.z = odomMsg.twist.twist.angular.z + GaussianNoise();
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