使用方法
    1.launchファイルの起動
        $ roslaunch get_joint_states get_joint_states.launch

    2.別のターミナルでrosbagコマンドを用いてtopicデータを保存
        $ rosbag record /**** -O data.bag

    3.launchを起動したターミナルで's'キーを押している間，データを保存

    4.rosbagをCtrl+Cで停止

    5.rosbagで取得したデータをcsvファイルに変換
        $ rostopic echo -b data.bag -p /**** > data.csv


依存関係
    1.xarm_ros/xarm_bringup/launch/xarm6_server.launch
