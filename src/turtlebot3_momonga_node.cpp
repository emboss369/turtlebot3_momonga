// ROSメインヘッダーファイル
// ROSプログラミングを行う際に必要となるROSファイルのインクルードを行う。
//後述するROS_INFO関数などを使用できるようになる。
#include <ros/ros.h>
#include <turtlebot3_msgs/SensorState.h>
// Cmakelist.txtでビルド後に自動的に生成されるように設定した
// メッセージファイルのヘッダーをインクルードする
#include "turtlebot3_momonga/msgBatteryLevel.h"

ros::Publisher pub;

// メッセージを受信した時に動作するコールバック関数を定義
void msgCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg){

    // シールドバッテリーの残量計算式
    // 残り容量(％)＝７０×電圧－８００
    float voltage = msg->battery;

    float level = 70.0 * voltage - 800.0;

    if (level > 100) level = 100;
    if (level < 0) level = 0;

    ROS_INFO("receieve battery:%f, %f",msg->battery,level);

    turtlebot3_momonga::msgBatteryLevel batteryLevel;
    batteryLevel.battery_level = level;
    pub.publish(batteryLevel);
}


int main (int argc, char** argv)
{

    ROS_INFO_STREAM("main");
    // Initialize ROS
    ros::init (argc, argv, "turtlebot3_battery");
    ros::NodeHandle nh;


    ros::Subscriber sub1 = nh.subscribe<turtlebot3_msgs::SensorState> ("sensor_state", 100, msgCallback);


    pub = nh.advertise<turtlebot3_momonga::msgBatteryLevel> ("battery_level", 100);


    // 0.1Hz(10秒に1回)
    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
