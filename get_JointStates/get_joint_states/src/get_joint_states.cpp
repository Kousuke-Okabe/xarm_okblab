#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <get_joint_states/Float64SaveData.h>

// #define DOF 6
#define Fs 10

static std::mutex g_mutex;

Eigen::VectorXd topic_position(6);
Eigen::VectorXd topic_velocity(6);
Eigen::VectorXd topic_effort(6);

int DOF;
char get_flag = 0;

// Callback function of the Topic communication with joint_states
void Subscribe_JointStates(const sensor_msgs::JointState &state){
    // lock the mutex
    std::lock_guard<std::mutex> lock(g_mutex);
    // ROS_INFO("subscribe joint_states");

    // ROS_INFO("Joint Effort : %f", state.effort[1]);
    for(int i=0; i<DOF; i++){
        topic_position(i) = (double)state.position[i];
        topic_velocity(i) = (double)state.position[i];
        topic_effort(i) = (double)state.effort[i];
    }
}

// Callback function of the Topic communication with Teleoperation(/cmd_vel)
void Subscribe_Teleop(const geometry_msgs::Twist &cmd){
    // lock the mutex
    std::lock_guard<std::mutex> lock(g_mutex);
    // ROS_INFO("subscribe teleop");

    if(cmd.linear.x == 0.0 && cmd.linear.y==0.0 && cmd.linear.z < -1 && cmd.angular.x == 0.0 && cmd.angular.y == 0.0 && cmd.angular.z == 0.0){
        get_flag = 1;
        ROS_INFO("subscribe teleop");
    }

}

int main(int argc, char **argv){
    using get_joint_states::Float64SaveData;

    // Initialize ROS node
    ros::init(argc, argv, "get_joint_satates");
    ros::NodeHandle nh;

    // Resistration the publishing topic
    ros::Publisher pub_save = nh.advertise<Float64SaveData>("save_data",10);

    // Resister the Callback function of Topic communication and setting
    ros::Subscriber sub_JointStates = nh.subscribe("/xarm/joint_states", 10, Subscribe_JointStates);
    ros::Subscriber sub_Teleop = nh.subscribe("cmd_vel", 10, Subscribe_Teleop);

    // Load Parameter
    DOF = nh.param<int>("/xarm/DOF", 6);

    // difinition the valiables
    Float64SaveData data;

    // Set loop rate
    ros::Rate rate(Fs);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        g_mutex.lock();

        // if press 's' key
        if(get_flag == 1){
            for(int i=0; i<DOF; i++){
                data.position[i] = topic_position(i);
                data.velocity[i] = topic_velocity(i);
                data.effort[i] = topic_effort(i);
            }

            // save data publish
            pub_save.publish(data);
        }

        get_flag = 0;

        g_mutex.unlock();
        rate.sleep();
    }

    return 0;
}