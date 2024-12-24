# xarm_control_continuance

xArm6に連続的な軌道を追従させる．
現在は追従させる起動をソースファイルで記述．

## 使い方
1. xArm_ros/xarm_bringup/launch/xarm6_server.launch を起動

   'roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.203'
   
3. xarm_okblab/xarm_control_continuance/xarm_control_continuance.launch を起動
   
   'roslaunch xarm_control_continuance xarm_control_continuance.launch'
