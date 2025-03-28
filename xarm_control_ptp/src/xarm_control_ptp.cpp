#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_control_ptp/SetAxis.h>
#include <xarm_control_ptp/SetInt16.h>
#include <xarm_control_ptp/Move.h>

int main(int argc, char** argv){
    using xarm_control_ptp::SetAxis;
    using xarm_control_ptp::SetInt16;
    using xarm_control_ptp::Move;

    // Initialize ROS
    ros::init(argc, argv, "xarm_control_ptp");
    ros::NodeHandle nh;

    // Resistration Service client
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");
    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_line");

    // Load ROS parameters
    float mvvelo = nh.param<float>("/xarm/mvvelo",1);
    float mvacc = nh.param<float>("/xarm/mvacc",1);

    float r_ref[6];
    r_ref[0] = nh.param<float>("/xarm/position_ref_x",250);
    r_ref[1] = nh.param<float>("/xarm/position_ref_y",0);
    r_ref[2] = nh.param<float>("/xarm/position_ref_z",350);
    r_ref[3] = nh.param<float>("/xarm/angle_ref_R",180);
    r_ref[4] = nh.param<float>("/xarm/angle_ref_P",200);
    r_ref[5] = nh.param<float>("/xarm/angle_ref_Y",2000);
    
    r_ref[3] = r_ref[3]/180*M_PI;
    r_ref[4] = r_ref[4]/180*M_PI;
    r_ref[5] = r_ref[5]/180*M_PI;

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
    IntData.request.data = 0;
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

    Move Move_command;
    std_msgs::Float32MultiArray position_command;
    position_command.data.resize(6);
    position_command.data[0] = r_ref[0];
    position_command.data[1] = r_ref[1];
    position_command.data[2] = r_ref[2];
    position_command.data[3] = r_ref[3];
    position_command.data[4] = r_ref[4];
    position_command.data[5] = r_ref[5];

    Move_command.request.pose = position_command.data;
    Move_command.request.mvvelo = mvvelo;
    Move_command.request.mvacc = mvacc;
    Move_command.request.mvtime = 0;
    Move_command.request.mvradii = 0;

    move_command.call(Move_command);
    if(Move_command.response.ret){
        ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
        return 1;
    }

    return 0;
}