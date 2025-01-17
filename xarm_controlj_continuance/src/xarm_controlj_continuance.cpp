#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_controlj_continuance/SetAxis.h>
#include <xarm_controlj_continuance/SetInt16.h>
#include <xarm_controlj_continuance/Move.h>

#define Fs      100

int main(int argc, char** argv){
    using xarm_controlj_continuance::SetAxis;
    using xarm_controlj_continuance::SetInt16;
    using xarm_controlj_continuance::Move;

    // Initialize ROS
    ros::init(argc, argv, "xarm_controlj_continuance");
    ros::NodeHandle nh;

    // Resistration Service client
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");
    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_joint");

    // Load ROS parameters
    float amplitude = nh.param<float>("/xarm/move_amplitude",M_PI/2);
    float period = nh.param<float>("/xarm/move_period",60);
    int move_joint = nh.param<int>("/xarm/move_joint",1);

    float q_init[6];
    q_init[0] = nh.param<float>("/xarm/initial_angle_1",0);
    q_init[1] = nh.param<float>("/xarm/initial_angle_2",0);
    q_init[2] = nh.param<float>("/xarm/initial_angle_3",0);
    q_init[3] = nh.param<float>("/xarm/initial_angle_4",0);
    q_init[4] = nh.param<float>("/xarm/initial_angle_5",0);
    q_init[5] = nh.param<float>("/xarm/initial_angle_6",0);
    
    q_init[0] = q_init[0]/180*M_PI;
    q_init[1] = q_init[1]/180*M_PI;
    q_init[2] = q_init[2]/180*M_PI;
    q_init[3] = q_init[3]/180*M_PI;
    q_init[4] = q_init[4]/180*M_PI;
    q_init[5] = q_init[5]/180*M_PI;

    // Servo ON
    SetAxis AxisData;
    AxisData.request.id = 8;
    AxisData.request.data = 1;
    motion_ctrl.call(AxisData);
    if(AxisData.response.ret){
        ROS_ERROR("Servo ON : %s", AxisData.response.message.c_str());
        return 1;
    }
    // ROS_INFO("Servo ON : id=%d, data=%d ", AxisData.request.id,AxisData.request.data);
    ROS_INFO("Servo ON : %s", AxisData.response.message.c_str());

    // Set control mode
    SetInt16 IntData;
    IntData.request.data = 6;
    set_mode.call(IntData);
    if(IntData.response.ret){
        ROS_ERROR("Set control mode : %s", IntData.response.message.c_str());
        return 1;
    }
    // ROS_INFO("Set control mode : data=%d", IntData.request.data);
    ROS_INFO("Set control mode : %s", IntData.response.message.c_str());

    // Set state normal
    IntData.request.data = 0;
    set_state.call(IntData);
    if(IntData.response.ret){
        ROS_ERROR("Set state : %s", IntData.response.message.c_str());
        return 1;
    }
    // ROS_INFO("Set state : data=%d", IntData.request.data);
    ROS_INFO("Set state : %s", IntData.response.message.c_str());

    // 周期設定
    double Ts = (double)1/Fs;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Time t0 = ros::Time::now();
    // ros::Time t = t0;
    ros::Rate rate(Fs); //[Hz]


    // Setting parameters
    Move Move_command;
    std_msgs::Float32MultiArray joint_command;
    joint_command.data.resize(6);
    joint_command.data[0] = q_init[0];
    joint_command.data[1] = q_init[1];
    joint_command.data[2] = q_init[2];
    joint_command.data[3] = q_init[3];
    joint_command.data[4] = q_init[4];
    joint_command.data[5] = q_init[5];

    Move_command.request.pose = joint_command.data;
    Move_command.request.mvvelo = 1000;
    Move_command.request.mvacc = 1000;
    Move_command.request.mvtime = 0;
    Move_command.request.mvradii = 0;

    while(ros::ok()){
        ros::Duration t = ros::Time::now() - t0;

        move_command.call(Move_command);
        if(Move_command.response.ret){
            ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
            break;
        }

        joint_command.data[move_joint-1] = q_init[move_joint-1] + amplitude/180*M_PI * sin(2*M_PI/period*t.toSec());

        Move_command.request.pose = joint_command.data;
        // Move_command.request.mvvelo = 200;
        // Move_command.request.mvacc = 2000;
        // Move_command.request.mvtime = 0;
        // Move_command.request.mvradii = 0;

        rate.sleep();
    }

    return 0;
}
