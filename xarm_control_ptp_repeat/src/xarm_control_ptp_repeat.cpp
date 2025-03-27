#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <xarm_control_ptp_repeat/SetAxis.h>
#include <xarm_control_ptp_repeat/SetInt16.h>
#include <xarm_control_ptp_repeat/Move.h>

#define NOREF 4

// sensor_msgs::JointState joint_states;
// std::mutex mut;
// bool sub_flag = false; 

int main(int argc, char** argv){
    using xarm_control_ptp_repeat::SetAxis;
    using xarm_control_ptp_repeat::SetInt16;
    using xarm_control_ptp_repeat::Move;

    // Initialize ROS
    ros::init(argc, argv, "xarm_control_ptp_repeat");
    ros::NodeHandle nh;

    // Resistration Service client
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");

    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_line");

    // Load ROS parameters
    float mvvelo = nh.param<float>("/xarm/mvvelo",1);
    float mvacc = nh.param<float>("/xarm/mvacc",1);

    float r_ref[NOREF][6];
    r_ref[0][0] = nh.param<float>("/xarm/angle_ref1_x",250);
    r_ref[0][1] = nh.param<float>("/xarm/angle_ref1_y",0);
    r_ref[0][2] = nh.param<float>("/xarm/angle_ref1_z",300);
    r_ref[0][3] = nh.param<float>("/xarm/angle_ref1_R",180);
    r_ref[0][4] = nh.param<float>("/xarm/angle_ref1_P",0);
    r_ref[0][5] = nh.param<float>("/xarm/angle_ref1_Y",0);

    r_ref[1][0] = nh.param<float>("/xarm/angle_ref2_x",250);
    r_ref[1][1] = nh.param<float>("/xarm/angle_ref2_y",0);
    r_ref[1][2] = nh.param<float>("/xarm/angle_ref2_z",400);
    r_ref[1][3] = nh.param<float>("/xarm/angle_ref2_R",180);
    r_ref[1][4] = nh.param<float>("/xarm/angle_ref2_P",0);
    r_ref[1][5] = nh.param<float>("/xarm/angle_ref2_Y",0);
    
    r_ref[2][0] = nh.param<float>("/xarm/angle_ref3_x",250);
    r_ref[2][1] = nh.param<float>("/xarm/angle_ref3_y",0);
    r_ref[2][2] = nh.param<float>("/xarm/angle_ref3_z",400);
    r_ref[2][3] = nh.param<float>("/xarm/angle_ref3_R",180);
    r_ref[2][4] = nh.param<float>("/xarm/angle_ref3_P",0);
    r_ref[2][5] = nh.param<float>("/xarm/angle_ref3_Y",0);
    
    r_ref[3][0] = nh.param<float>("/xarm/angle_ref4_x",250);
    r_ref[3][1] = nh.param<float>("/xarm/angle_ref4_y",0);
    r_ref[3][2] = nh.param<float>("/xarm/angle_ref4_z",400);
    r_ref[3][3] = nh.param<float>("/xarm/angle_ref4_R",180);
    r_ref[3][4] = nh.param<float>("/xarm/angle_ref4_P",0);
    r_ref[3][5] = nh.param<float>("/xarm/angle_ref4_Y",0);
    
    for(int i=0; i<NOREF; i++){
        for(int j=3; j<6; j++){
            r_ref[i][j] = r_ref[i][j]/180*M_PI;
        }
    }

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
    position_command.data.resize(NOREF);
    for(int j=0; j<6; j++){
        position_command.data[j] = r_ref[0][j];
    }

    Move_command.request.pose = position_command.data;
    Move_command.request.mvvelo = mvvelo;
    Move_command.request.mvacc = mvacc;
    Move_command.request.mvtime = 0;
    Move_command.request.mvradii = 0;

    bool move_flag = true;
    int ref_cnt = 0;

    // move_command.call(Move_command);
    // if(Move_command.response.ret){
    //     ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
    //     return 1;
    // }

    ros::Rate loop_rate(0.2);

    while(ros::ok()){
        ROS_INFO("MOVE ref%d", ref_cnt);
        for(int i=0; i<6; i++){
            position_command.data[i] = r_ref[ref_cnt][i];
        }

        if(move_flag){
            if(ref_cnt < NOREF-2)
                ref_cnt ++;
            else{
                move_flag = false;
                ref_cnt --;
            }
        }
        else{
            if(ref_cnt > 0)
                ref_cnt --;
            else{
                move_flag = true;
                ref_cnt ++;
            }
        }

        // if(move_flag){
        //     ROS_INFO("MOVE ref1");

        //     position_command.data[0] = r_ref1[0];
        //     position_command.data[1] = r_ref1[1];
        //     position_command.data[2] = r_ref1[2];
        //     position_command.data[3] = r_ref1[3];
        //     position_command.data[4] = r_ref1[4];
        //     position_command.data[5] = r_ref1[5];

        //     move_flag = false;
        // }
        // else{
        //     ROS_INFO("MOVE ref2");

        //     position_command.data[0] = r_ref2[0];
        //     position_command.data[1] = r_ref2[1];
        //     position_command.data[2] = r_ref2[2];
        //     position_command.data[3] = r_ref2[3];
        //     position_command.data[4] = r_ref2[4];
        //     position_command.data[5] = r_ref2[5];

        //     move_flag = true;
        // }

        Move_command.request.pose = position_command.data;
        
        move_command.call(Move_command);
        if(Move_command.response.ret){
            ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
            return 1;
        }

        // loop_rate.sleep();
    }

    return 0;
}