#include <ros/ros.h>
#include <xarm_control/SetAxis.h>
#include <xarm_control/SetInt16.h>

int main(int argc, char** argv){
    using xarm_control::SetAxis;
    using xarm_control::SetInt16;

    // Initialize ROS
    ros::init(argc, argv, "xarm_control");
    ros::NodeHandle nh;

    // Resistration Service
    ros::ServiceClient motion_ctrl = nh.serviceClient<SetAxis>("/xarm/motion_ctrl");
    ros::ServiceClient set_mode = nh.serviceClient<SetInt16>("/xarm/set_mode");
    ros::ServiceClient set_state = nh.serviceClient<SetInt16>("/xarm/set_state");

    // Servo ON
    SetAxis AxisData;
    AxisData.request.id = 8;
    AxisData.request.data = 1;
    motion_ctrl.call(AxisData);
    if(AxisData.response.ret){
        ROS_ERROR("Servo ON error : %s", AxisData.response.message.c_str());
        return 1;
    }

    // Set control mode
    SetInt16 IntData;
    IntData.request.data = 1;
    set_mode.call(IntData);
    if(IntData.response.ret){
        ROS_ERROR("Set control mode error : %s", IntData.response.message.c_str());
        return 1;
    }

    // Set state normal
    IntData.request.data = 0;
    set_state.call(IntData);
    if(IntData.response.ret){
        ROS_ERROR("Set state error : %s", IntData.response.message.c_str());
        return 1;
    }

    ROS_INFO("Success");

}