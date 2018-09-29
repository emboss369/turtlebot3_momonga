// ROSメインヘッダーファイル
// ROSプログラミングを行う際に必要となるROSファイルのインクルードを行う。
//後述するROS_INFO関数などを使用できるようになる。
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
// Cmakelist.txtでビルド後に自動的に生成されるように設定した
// メッセージファイルのヘッダーをインクルードする
#include <std_msgs/Float32.h>

ros::Publisher pub;
ros::Publisher vpub;

// メッセージを受信した時に動作するコールバック関数を定義
void msgCallback(const sensor_msgs::BatteryState::ConstPtr &msg){

    //ROS_INFO("receieve battery:%f, %f",msg->voltage,msg->percentage);

    std_msgs::Float32 batteryLevel;
    batteryLevel.data = msg->percentage;
    pub.publish(batteryLevel);
    std_msgs::Float32 batteryVoltage;
    batteryVoltage.data = msg->voltage;
    vpub.publish(batteryVoltage);
}


int main (int argc, char** argv)
{

    ROS_INFO_STREAM("main");
    // Initialize ROS
    ros::init (argc, argv, "turtlebot3_battery");
    ros::NodeHandle nh;


    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::BatteryState> ("battery_state", 100, msgCallback);


    pub = nh.advertise<std_msgs::Float32> ("battery_level", 100);
    vpub = nh.advertise<std_msgs::Float32> ("battery_voltage", 100);


    ros::spin();

    return 0;
}
