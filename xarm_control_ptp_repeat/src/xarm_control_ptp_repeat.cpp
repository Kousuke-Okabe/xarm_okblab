#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <xarm_control_ptp_repeat/SetAxis.h>
#include <xarm_control_ptp_repeat/SetInt16.h>
#include <xarm_control_ptp_repeat/Move.h>

// sensor_msgs::JointState joint_states;
// std::mutex mut;
// bool sub_flag = false; 

// void Subscribe_jointstates(const sensor_msgs::JointState::ConstPtr& msg){
//     std::lock_guard<std::mutex> lock(mut);

//     joint_states.position.resize(msg->position.size());
//     joint_states.velocity.resize(msg->velocity.size());
//     joint_states.effort.resize(msg->effort.size());
//     joint_states.name.resize(msg->name.size());
//     joint_states.position = msg->position;
//     joint_states.velocity = msg->velocity;
//     joint_states.effort = msg->effort;
//     joint_states.name = msg->name;

//     sub_flag = true;
// }

int main(int argc, char** argv){
    using xarm_control_ptp_repeat::SetAxis;
    using xarm_control_ptp_repeat::SetInt16;
    using xarm_control_ptp_repeat::Move;

    // Initialize ROS //////////////////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "xarm_control_ptp_repeat");
    ros::NodeHandle nh;

    // Resistration Publisher ///////////////////////////////////////////////////////////////////////
    // ros::Publisher pub_jointstates_rosbag = nh.advertise<sensor_msgs::JointState>("/xarm/joint_states_rosbag", 10);

    // Resistration callback function of subscribe //////////////////////////////////////////////////
    // ros::Subscriber sub_jointstates = nh.subscribe("/xarm/joint_states", 10, Subscribe_jointstates);

    // Resistration Service client //////////////////////////////////////////////////////////////////
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");

    ros::ServiceClient move_command = nh.serviceClient<Move>("/xarm/move_line");

    // Load ROS parameters //////////////////////////////////////////////////////////////////////////
    float mvvelo = nh.param<float>("/xarm/mvvelo",1);
    float mvacc = nh.param<float>("/xarm/mvacc",1);

    float r_ref1[6];
    r_ref1[0] = nh.param<float>("/xarm/angle_ref1_x",250);
    r_ref1[1] = nh.param<float>("/xarm/angle_ref1_y",0);
    r_ref1[2] = nh.param<float>("/xarm/angle_ref1_z",300);
    r_ref1[3] = nh.param<float>("/xarm/angle_ref1_R",180);
    r_ref1[4] = nh.param<float>("/xarm/angle_ref1_P",0);
    r_ref1[5] = nh.param<float>("/xarm/angle_ref1_Y",0);

    float r_ref2[6];
    r_ref2[0] = nh.param<float>("/xarm/angle_ref2_x",250);
    r_ref2[1] = nh.param<float>("/xarm/angle_ref2_y",0);
    r_ref2[2] = nh.param<float>("/xarm/angle_ref2_z",400);
    r_ref2[3] = nh.param<float>("/xarm/angle_ref2_R",180);
    r_ref2[4] = nh.param<float>("/xarm/angle_ref2_P",0);
    r_ref2[5] = nh.param<float>("/xarm/angle_ref2_Y",0);
    
    r_ref1[3] = r_ref1[3]/180*M_PI;
    r_ref1[4] = r_ref1[4]/180*M_PI;
    r_ref1[5] = r_ref1[5]/180*M_PI;

    r_ref2[3] = r_ref2[3]/180*M_PI;
    r_ref2[4] = r_ref2[4]/180*M_PI;
    r_ref2[5] = r_ref2[5]/180*M_PI;

    // Servo ON ///////////////////////////////////////////////////////////////////////////////////
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

    // Set control mode ///////////////////////////////////////////////////////////////////////////
    SetInt16 IntData;
    IntData.request.data = 0;
    set_mode.call(IntData);
    if(IntData.response.ret){
        ROS_ERROR("Set control mode : %s", IntData.response.message.c_str());
        return 1;
    }
    // ROS_INFO("Set control mode : data=%d", IntData.request.data);
    ROS_INFO("Set control mode : %s", IntData.response.message.c_str());

    // Set state normal ///////////////////////////////////////////////////////////////////////////
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
    position_command.data[0] = r_ref1[0];
    position_command.data[1] = r_ref1[1];
    position_command.data[2] = r_ref1[2];
    position_command.data[3] = r_ref1[3];
    position_command.data[4] = r_ref1[4];
    position_command.data[5] = r_ref1[5];

    Move_command.request.pose = position_command.data;
    Move_command.request.mvvelo = mvvelo;
    Move_command.request.mvacc = mvacc;
    Move_command.request.mvtime = 0;
    Move_command.request.mvradii = 0;

    bool move_flag = false;

    // move_command.call(Move_command);
    // if(Move_command.response.ret){
    //     ROS_ERROR("Call error : %s", Move_command.response.message.c_str());
    //     return 1;
    // }

    ros::Rate loop_rate(0.2);

    while(ros::ok()){
        if(move_flag){
            ROS_INFO("MOVE ref1");

            position_command.data[0] = r_ref1[0];
            position_command.data[1] = r_ref1[1];
            position_command.data[2] = r_ref1[2];
            position_command.data[3] = r_ref1[3];
            position_command.data[4] = r_ref1[4];
            position_command.data[5] = r_ref1[5];

            move_flag = false;
        }
        else{
            ROS_INFO("MOVE ref2");

            position_command.data[0] = r_ref2[0];
            position_command.data[1] = r_ref2[1];
            position_command.data[2] = r_ref2[2];
            position_command.data[3] = r_ref2[3];
            position_command.data[4] = r_ref2[4];
            position_command.data[5] = r_ref2[5];

            move_flag = true;
        }

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