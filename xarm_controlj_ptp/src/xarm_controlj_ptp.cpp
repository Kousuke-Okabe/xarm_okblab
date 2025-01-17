#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <xarm_controlj_ptp/SetAxis.h>
#include <xarm_controlj_ptp/SetInt16.h>
#include <xarm_controlj_ptp/Move.h>

int main(int argc, char** argv){
    using xarm_controlj_ptp::SetAxis;
    using xarm_controlj_ptp::SetInt16;
    using xarm_controlj_ptp::Move;

    // Initialize ROS
    ros::init(argc, argv, "xarm_controlj_ptp");
    ros::NodeHandle nh;

    // Resistration Service client
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");
    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_joint");

    // Load ROS parameters
    float mvvelo = nh.param<float>("/xarm/mvvelo",1);
    float mvacc = nh.param<float>("/xarm/mvacc",1);

    float q_ref[6];
    q_ref[0] = nh.param<float>("/xarm/angle_ref_1",0);
    q_ref[1] = nh.param<float>("/xarm/angle_ref_2",0);
    q_ref[2] = nh.param<float>("/xarm/angle_ref_3",0);
    q_ref[3] = nh.param<float>("/xarm/angle_ref_4",0);
    q_ref[4] = nh.param<float>("/xarm/angle_ref_5",0);
    q_ref[5] = nh.param<float>("/xarm/angle_ref_6",0);
    
    q_ref[0] = q_ref[0]/180*M_PI;
    q_ref[1] = q_ref[1]/180*M_PI;
    q_ref[2] = q_ref[2]/180*M_PI;
    q_ref[3] = q_ref[3]/180*M_PI;
    q_ref[4] = q_ref[4]/180*M_PI;
    q_ref[5] = q_ref[5]/180*M_PI;

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
    position_command.data[0] = q_ref[0];
    position_command.data[1] = q_ref[1];
    position_command.data[2] = q_ref[2];
    position_command.data[3] = q_ref[3];
    position_command.data[4] = q_ref[4];
    position_command.data[5] = q_ref[5];

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