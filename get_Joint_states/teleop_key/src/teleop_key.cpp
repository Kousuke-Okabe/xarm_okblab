#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_SPACE 0x20

using namespace std;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle()://コンストラクタ
  linear_(0),
  angular_(0),
  l_scale_(1.1),
  a_scale_(1.1)
{
  nh_.param("scale_angular", a_scale_, a_scale_);//pram,default
  nh_.param("scale_linear", l_scale_, l_scale_);//pram,default

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

int kfd = 0;
struct termios cooked, raw;//非同期通信ポート制御
//端末を細かく制御するための構造体で、termio.hで定義される。
//この構造体で定義されている関数を呼び出すためには、cursesもしくはncursesライブラリとリンクして用いる。すなわち、コンパイルするときに -ncursesもしくは　-lncursesをつけてコンパイルする。


void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_key");//ノード生成
  TeleopTurtle teleop_turtle;

  //割り込み処理
  signal(SIGINT,quit);//SIGINT:Ctrl+c,実行関数

  teleop_turtle.keyLoop();
  
  return(0);
}

//メイン動作
void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  //ポート設定
  //kfdパラメータを取得し、参照構造体termiosに設定する。成功:0,失敗:-1が帰り、errnoがセットされる。
  tcgetattr(kfd, &cooked);//fd,termios
  
  //メモリブロックのコピー
  memcpy(&raw, &cooked, sizeof(struct termios));//コピー先、コピー元、コピーバイト数

  //ローカルモード
  raw.c_lflag &=~ (ICANON | ECHO);//正規モード、入力文字をエコーする
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  //端末に関連したパラメータを設定する
  //TCSCANOW:値を直ちに変更
  tcsetattr(kfd, TCSANOW, &raw);//fd,action,termios

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  int i;
  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
	
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 zero;
	zero.x = zero.y = zero.z = 0.0;
    switch(c)
    {
      case KEYCODE_A:
        ROS_DEBUG("LEFT");
		cout << "LEFT" << endl;
        twist.linear.y = l_scale_*1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT");
		cout << "RIGHT" << endl;
        twist.linear.y = l_scale_*-1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("FRONT");
		cout << "FRONT" << endl;
        twist.linear.x = l_scale_*1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("BACK");
		cout << "BACK" << endl;
        twist.linear.x = l_scale_*-1.0;
        dirty = true;
        break;
	  case KEYCODE_W:
        ROS_DEBUG("UP");
		cout << "UP" << endl;
        twist.linear.z = l_scale_*1.0;
        dirty = true;
        break;
	  case KEYCODE_S:
        ROS_DEBUG("DOWN");
		cout << "DOWN" << endl;
        twist.linear.z = l_scale_*-1.0;
        dirty = true;
        break;
	  case KEYCODE_L:
        ROS_DEBUG("YAW_L");
		cout << "YAW_L" << endl;
        twist.angular.z = a_scale_*1.0;
        dirty = true;
        break;
	  case KEYCODE_R:
        ROS_DEBUG("YAW_R");
		cout << "YAW_R" << endl;
        twist.angular.z = a_scale_*-1.0;
        dirty = true;
        break;
	  case KEYCODE_SPACE:
		ROS_DEBUG("STOP");
		cout << "STOP" << endl;
        twist.linear = zero;
		twist.angular = zero;
        dirty = true;
        break;
    }
   

    
    //twist.angular.z = a_scale_*angular_;
    //twist.linear.z = l_scale_*linear_;
    if(/*dirty ==*/true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}


