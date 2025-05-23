#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_control_continuance/SetAxis.h>
#include <xarm_control_continuance/SetInt16.h>
#include <xarm_control_continuance/Move.h>

#define Fs      100

int main(int argc, char** argv){
    using xarm_control_continuance::SetAxis;
    using xarm_control_continuance::SetInt16;
    using xarm_control_continuance::Move;

    // Initialize ROS
    ros::init(argc, argv, "xarm_control_continuance");
    ros::NodeHandle nh;

    // Resistration Service client
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");
    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_servo_cart");

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
    IntData.request.data = 1;
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

    Move Move_command;
    std_msgs::Float32MultiArray position_command;
    position_command.data.resize(6);
    position_command.data[0] = 250;
    position_command.data[1] = 0;
    position_command.data[2] = 300;
    position_command.data[3] = M_PI;
    position_command.data[4] = 0;
    position_command.data[5] = 0;

    Move_command.request.pose = position_command.data;
    Move_command.request.mvvelo = 200;
    Move_command.request.mvacc = 2000;
    Move_command.request.mvtime = 0;
    Move_command.request.mvradii = 0;

    // move_command.call(Move_command);
    // if(Move_command.response.ret){
    //     ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
    //     return 1;
    // }

    while(ros::ok()){
        ros::Duration t = ros::Time::now() - t0;

        move_command.call(Move_command);
        if(Move_command.response.ret){
            ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
            break;
        }

        position_command.data[0] = 350 + 100 * sin(2*M_PI*0.1*t.toSec());
        position_command.data[1] =   0 + 100 * cos(2*M_PI*0.1*t.toSec());

        Move_command.request.pose = position_command.data;
        // Move_command.request.mvvelo = 200;
        // Move_command.request.mvacc = 2000;
        // Move_command.request.mvtime = 0;
        // Move_command.request.mvradii = 0;

        rate.sleep();
    }

    return 0;
}
