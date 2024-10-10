#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <Eigen/Dense>
# include <bits/stdc++.h>   // for M_PI

#define DOF 6
#define Fs 10

static std::mutex g_mutex;

Eigen::VectorXd present_angle(6);
Eigen::VectorXd present_torque(6);
Eigen::MatrixXd jacobi(3,6), Jp(6,3);
Eigen::Matrix3d JJt;
Eigen::VectorXd Us(6);
Eigen::MatrixXd E =  Eigen::MatrixXd::Identity(6,6);

// Callback function of the Topic communication with joint_states
void Subscribe_JointStates(const sensor_msgs::JointState &state){
    // lock the mutex
    std::lock_guard<std::mutex> lock(g_mutex);

    // ROS_INFO("Joint Effort : %f", state.effort[1]);
    for(int i=0; i<DOF; i++){
        present_angle(i) = (double)state.position[i];
        present_torque(i) = (double)state.effort[i];
    }
}

int main(int argc, char **argv){
    // Initialize ROS node
    ros::init(argc, argv, "xarm6_collision");
    ros::NodeHandle nh;

    // Resister the Callback function of Topic communication and setting
    ros::Subscriber sub = nh.subscribe("/xarm/joint_states", 10, Subscribe_JointStates);

    // Load Parameter
    double Sigma = nh.param<double>("Threshold_of_CollisionDetection",0.1);
    double d1 = nh.param<double>("Modified_DH_d1", 0.267);
    double d4 = nh.param<double>("Modified_DH_d4", 0.3425);
    double d6 = nh.param<double>("Modified_DH_d6", 0.097);
    double a3 = nh.param<double>("Modified_DH_a3", 0.28948866);
    double a4 = nh.param<double>("Modified_DH_a4", 77.5);
    double a6 = nh.param<double>("Modified_DH_a6", 76);
    double T2_off = nh.param<double>("Modified_DH_T2offset", -1.3849179);
    double T3_off = nh.param<double>("Modified_DH_T3offset", 1.3849179);

    // Set loop rate
    ros::Rate rate(Fs);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        // ros::spinOnce();
        g_mutex.lock();

        // Jacobian matrix with dq to r
        jacobi(0,0) = (-d6*cos(present_angle(1))*sin(present_angle(4))+(d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*cos(present_angle(4))+a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+(a6*cos(present_angle(1))*sin(present_angle(4))+(a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(a4*sin(present_angle(1))*sin(present_angle(2)+T2_off)+d4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(d4*sin(present_angle(1))*sin(present_angle(2)+T2_off)-a4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off)-a3*sin(present_angle(1))*cos(present_angle(2)+T2_off);
        jacobi(1,0) = (-d6*sin(present_angle(1))*sin(present_angle(4))+(d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+(a6*sin(present_angle(1))*sin(present_angle(4))+(a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*cos(present_angle(4))-d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(-a4*cos(present_angle(1))*sin(present_angle(2)+T2_off)-d4*cos(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(a4*cos(present_angle(1))*cos(present_angle(2)+T2_off)-d4*cos(present_angle(1))*sin(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off)+a3*cos(present_angle(1))*cos(present_angle(2)+T2_off);
        jacobi(2,0) = 0;

        jacobi(0,1) = ((d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(d4*cos(present_angle(1))*sin(present_angle(2)+T2_off)-a4*cos(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(-a4*cos(present_angle(1))*sin(present_angle(2)+T2_off)-d4*cos(present_angle(1))*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off)-a3*cos(present_angle(1))*sin(present_angle(2)+T2_off);
        jacobi(1,1) = ((d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(d4*sin(present_angle(1))*sin(present_angle(2)+T2_off)-a4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(-a4*sin(present_angle(1))*sin(present_angle(2)+T2_off)-d4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off)-a3*sin(present_angle(1))*sin(present_angle(2)+T2_off);
        jacobi(2,1) = ((d6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-d6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*cos(present_angle(4))+a6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+a6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((a6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(a4*sin(present_angle(2)+T2_off)+d4*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(d4*sin(present_angle(2)+T2_off)-a4*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off)-a3*cos(present_angle(2)+T2_off);

        jacobi(0,2) = ((d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(d4*cos(present_angle(1))*sin(present_angle(2)+T2_off)-a4*cos(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(-a4*cos(present_angle(1))*sin(present_angle(2)+T2_off)-d4*cos(present_angle(1))*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off);
        jacobi(1,2) = ((d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(d4*sin(present_angle(1))*sin(present_angle(2)+T2_off)-a4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(-a4*sin(present_angle(1))*sin(present_angle(2)+T2_off)-d4*sin(present_angle(1))*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off);
        jacobi(2,2) = ((d6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-d6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*cos(present_angle(4))+a6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+a6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((a6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5))+(a4*sin(present_angle(2)+T2_off)+d4*cos(present_angle(2)+T2_off))*sin(present_angle(3)+T3_off)+(d4*sin(present_angle(2)+T2_off)-a4*cos(present_angle(2)+T2_off))*cos(present_angle(3)+T3_off);

        jacobi(0,3) = ((d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*sin(present_angle(4))-d6*sin(present_angle(1))*cos(present_angle(4)))*sin(present_angle(5))+((a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(4))+a6*sin(present_angle(1))*cos(present_angle(4)))*cos(present_angle(5));
        jacobi(1,3) = ((d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off)-d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off))*sin(present_angle(4))+d6*cos(present_angle(1))*cos(present_angle(4)))*sin(present_angle(5))+((a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(4))-a6*cos(present_angle(1))*cos(present_angle(4)))*cos(present_angle(5));
        jacobi(2,3) = (-d6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(4))*sin(present_angle(5))+(a6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+a6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(4))*cos(present_angle(5));

        jacobi(0,4) = (-a6*sin(present_angle(1))*sin(present_angle(4))+(a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+(-d6*sin(present_angle(1))*sin(present_angle(4))+(d6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))-a6*cos(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5));
        jacobi(1,4) = (a6*cos(present_angle(1))*sin(present_angle(4))+(a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+(d6*cos(present_angle(1))*sin(present_angle(4))+(d6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-d6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))-a6*sin(present_angle(1))*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*sin(present_angle(1))*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5));
        jacobi(2,4) = ((a6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+a6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))-d6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*sin(present_angle(5))+((d6*cos(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)+d6*sin(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(4))+a6*sin(present_angle(2)+T2_off)*sin(present_angle(3)+T3_off)-a6*cos(present_angle(2)+T2_off)*cos(present_angle(3)+T3_off))*cos(present_angle(5));

        jacobi(0,5) = 0;
        jacobi(1,5) = 0;
        jacobi(2,5) = 0;

        // Collisition detection
        JJt = jacobi * jacobi.transpose();
        Jp = jacobi.transpose() * JJt.inverse();

        Us = (E - Jp * jacobi) * present_torque;

        if(Us.norm() > Sigma)   ROS_INFO("||Us||=%7.3f> %2.1f  Collision", (double)Us.norm(),Sigma);
        else                    ROS_INFO("||Us||=%7.3f<=%2.1f", (double)Us.norm(),Sigma);


        // ROS_INFO("effort_0 : %f", present_torque(1));

        g_mutex.unlock();
        rate.sleep();
    }

    // ros::spin();
    return 0;
}