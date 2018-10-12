#include <ros/ros.h>
#include <ros/package.h> // パッケージの検索
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tf/transform_listener.h>

#define ROS_GREEN_STREAM(x) ROS_INFO_STREAM("\033[1;32m" << x << "\033[0m")
#define WAYPOINT_TYPE_NORMAL 0
#define WAYPOINT_TYPE_STOP 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

// ロボットの状態を表す
namespace RobotBehaviors
{
enum State
{
    WAYPOINT_NAV,                 // Waypointに向かって進行中
    WAYPOINT_REACHED_GOAL,        // Waypointに到着しました
    WAYPOINT_REACHED_GOAL_WAIT,   // Waypointに到着して再開を待機中です
    INIT_NAV,                     // ナビゲータ初期化
    WAYPOINT_NAV_PLANNING_ABORTED // WAYPOINT_NAVから移動しようとしたが移動していないようだ。プランニング失敗。
};
}

// 1このWaypointを表すクラス
class WayPoint
{
  public:
    WayPoint();
    WayPoint(move_base_msgs::MoveBaseGoal goal, int waypoint_type, double reach_threshold)
        : goal_(goal), waypoint_type_(waypoint_type), reach_threshold_(reach_threshold)
    {
    }
    move_base_msgs::MoveBaseGoal goal_;
    int waypoint_type_;
    double reach_threshold_;
};

// Waypointに近くなったら次のポイントをゴールとして設定します
class WaypointNavigator
{
  public:
    // コンストラクタ
    WaypointNavigator()
        : ac_("move_base", true),
          rate_(10)
    {
        target_waypoint_index_ = 0; // 次に目指すウェイポイントの初期化
        robot_behavior_state_ = RobotBehaviors::INIT_NAV;
        std::string filename; // Waypoint一覧CSVファイル名
        ros::NodeHandle n("~");
        n.param<std::string>("waypointsfile",
                             filename,
                             ros::package::getPath("turtlebot3_momonga") + "/waypoints/waypoints.csv");

        ROS_INFO("[Waypoints file name] : %s", filename.c_str());

        // 目標となるWaypointの表示用
        next_waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/next_waypoint", 1);

        // 一時停止からの再開始時を受け取る
        resume_sub_ = nh_.subscribe("/resume_waypoint", 1, &WaypointNavigator::resumeCallback, this);

        ROS_INFO("Reading Waypoints.");
        readWaypoint(filename.c_str());
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
    }

    // デストラクタでゴールをキャンセルして停止。
    ~WaypointNavigator()
    {
        this->cancelGoal();
    }

    int readWaypoint(std::string filename)
    {
        const int rows_num = 9; // x, y, z, Qx,Qy,Qz,Qw, waypoint_type, reach_threshold
        boost::char_separator<char> sep(",", "", boost::keep_empty_tokens);
        std::ifstream ifs(filename.c_str());
        std::string line;
        while (ifs.good())
        {
            getline(ifs, line);
            if (line.empty())
                break;
            tokenizer tokens(line, sep);
            std::vector<double> data;
            tokenizer::iterator it = tokens.begin();
            for (; it != tokens.end(); ++it)
            {
                std::stringstream ss;
                double d;
                ss << *it;
                ss >> d;
                data.push_back(d);
            }
            if (data.size() != rows_num)
            {
                ROS_ERROR("Row size is mismatch!!");
                return -1;
            }
            else
            {
                move_base_msgs::MoveBaseGoal waypoint;
                waypoint.target_pose.pose.position.x = data[0];
                waypoint.target_pose.pose.position.y = data[1];
                waypoint.target_pose.pose.position.z = data[2];
                waypoint.target_pose.pose.orientation.x = data[3];
                waypoint.target_pose.pose.orientation.y = data[4];
                waypoint.target_pose.pose.orientation.z = data[5];
                waypoint.target_pose.pose.orientation.w = data[6];
                waypoints_.push_back(WayPoint(waypoint, (int)data[7], data[8] / 2.0));
            }
        }
        return 0;
    }

    // ゴールをキャンセルして停止します
    void cancelGoal()
    {
        ROS_INFO("cancelGoal() is called !!");
        ac_.cancelGoal();
    }

    // 次のWaypointを返します
    WayPoint getNextWaypoint()
    {
        ROS_INFO_STREAM("Next Waypoint : " << target_waypoint_index_);
        WayPoint next_waypoint = waypoints_[target_waypoint_index_];
        target_waypoint_index_++; // その次のターゲットに進める
        return next_waypoint;
    }

    // WaypointMakerを画面上に表示する
    void sendNextWaypointMarker(const geometry_msgs::Pose waypoint,
                                int waypoint_type)
    {
        visualization_msgs::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "map";
        waypoint_marker.header.stamp = ros::Time();
        waypoint_marker.id = 0;
        waypoint_marker.type = visualization_msgs::Marker::ARROW;
        waypoint_marker.action = visualization_msgs::Marker::ADD;
        waypoint_marker.pose = waypoint;
        waypoint_marker.pose.position.z = 0.3;
        waypoint_marker.scale.x = 0.8;
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.1;
        waypoint_marker.color.a = 0.7;
        waypoint_marker.color.r = 0.0;
        waypoint_marker.color.g = 0.0;
        waypoint_marker.color.b = 0.0;
        if (waypoint_type == WAYPOINT_TYPE_NORMAL)
        {
            waypoint_marker.color.g = 0.80;
        }
        else if (waypoint_type == WAYPOINT_TYPE_STOP)
        {
            waypoint_marker.color.r = 0.80;
        }
        next_waypoint_marker_pub_.publish(waypoint_marker);
    }

    // 新しいゴール地点を move_baseに渡す
    void sendNewGoal(geometry_msgs::Pose pose)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose = pose;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ac_.sendGoal(goal);
        now_goal_ = goal.target_pose.pose;
    }
    // 通常のwaypointの場合
    void setNextGoal(WayPoint waypoint)
    {
        reach_threshold_ = waypoint.reach_threshold_;
        this->sendNextWaypointMarker(waypoint.goal_.target_pose.pose, waypoint.waypoint_type_); // 現在目指しているwaypointを表示する
        this->sendNewGoal(waypoint.goal_.target_pose.pose);
    }

    // 現在のロボットの位置を返す
    geometry_msgs::Pose getRobotCurrentPosition()
    {
        // tfを使ってロボットの現在位置を取得する
        tf::StampedTransform transform;
        geometry_msgs::Pose pose;
        try
        {
            listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        //ROS_INFO_STREAM("c)x :" << pose.position.x << ", y :" << pose.position.y);
        return pose;
    }

    // 現在のロボットの位置
    geometry_msgs::Pose getNowGoalPosition()
    {
        //ROS_INFO_STREAM("g)x :" << now_goal_.position.x << ", y :" << now_goal_.position.y);
        return now_goal_;
    }

    // aとbの２点間の距離を返す
    double calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
    {
        return sqrt(pow((a.position.x - b.position.x), 2.0) + pow((a.position.y - b.position.y), 2.0));
    }

    // 今セットされてるゴールへのしきい値を返す関数
    double getReachThreshold()
    {
        return reach_threshold_;
    }

    // 最後のWaypointだったか？
    bool isFinalGoal()
    {
        if ((target_waypoint_index_) == ((int)waypoints_.size()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void resumeCallback(const std_msgs::Empty &toggle_msg)
    {
        ROS_INFO("RESUME CALLBACK");
        resume_ = true;
    }

    void run()
    {
        robot_behavior_state_ = RobotBehaviors::INIT_NAV;
        resume_ = false;
        while (ros::ok())
        {
            WayPoint next_waypoint = this->getNextWaypoint();
            ROS_GREEN_STREAM("Next WayPoint is got");
            // if (next_waypoint.waypoint_type_ == WAYPOINT_TYPE_NORMAL)
            // {
                ROS_INFO("Go next_waypoint.");
                this->setNextGoal(next_waypoint);
                robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV;
            // }

            ros::Time begin_navigation = ros::Time::now(); // 新しいナビゲーションを設定した時間
            ros::Time verbose_start = ros::Time::now();    //  定期的に進捗報告する（30秒ごとに報告）

            double last_distance_to_goal = 0;
            double delta_distance_to_goal = 1.0; // 0.1[m]より大きければよい

            while (ros::ok())
            {
                geometry_msgs::Pose robot_current_position = this->getRobotCurrentPosition();                 // 現在のロボットの座標
                geometry_msgs::Pose now_goal_position = this->getNowGoalPosition();                           // 現在目指している座標
                double distance_to_goal = this->calculateDistance(robot_current_position, now_goal_position); // 現在位置とwaypointまでの距離を計算

                // ここからスタック(Abort)判定。
                delta_distance_to_goal = last_distance_to_goal - distance_to_goal; // どれだけ進んだか
                if (delta_distance_to_goal < 0.1)
                { // 進んだ距離が0.1[m]より小さくて
                    ros::Duration how_long_stay_time = ros::Time::now() - begin_navigation;
                    if (how_long_stay_time.toSec() > 300.0)
                    { // 300秒間経過していたら
                        if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV)
                        {
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED; // プランニング失敗とする
                            break;
                        }
                    }
                    else
                    { // 30秒おきに進捗を報告する
                        ros::Duration verbose_time = ros::Time::now() - verbose_start;
                        if (verbose_time.toSec() > 30.0)
                        {
                            ROS_INFO_STREAM("Waiting Abort: passed 30s, Distance to goal: " << distance_to_goal);
                            verbose_start = ros::Time::now();
                        }
                    }
                }
                else
                { // 0.1[m]以上進んでいればOK
                    last_distance_to_goal = distance_to_goal;
                    begin_navigation = ros::Time::now();
                }

                // waypointの更新判定
                if (distance_to_goal < this->getReachThreshold())
                { // 目標座標までの距離がしきい値になれば
                    ROS_INFO_STREAM("Distance: " << distance_to_goal);
                    if (robot_behavior_state_ == RobotBehaviors::WAYPOINT_NAV)
                    {
                        if (next_waypoint.waypoint_type_ == WAYPOINT_TYPE_NORMAL)
                        {
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_GOAL;
                        }
                        else if (next_waypoint.waypoint_type_ == WAYPOINT_TYPE_STOP)
                        { // 一時停止Waypointならば
                            robot_behavior_state_ = RobotBehaviors::WAYPOINT_REACHED_GOAL_WAIT;
                        }

                        
                    }

                    break;
                }

                rate_.sleep();
                ros::spinOnce();
            }

            switch (robot_behavior_state_)
            {
            case RobotBehaviors::WAYPOINT_REACHED_GOAL:
            {
                ROS_INFO("WAYPOINT_REACHED_GOAL");
                if (this->isFinalGoal())
                {                       // そのwaypointが最後だったら
                    this->cancelGoal(); // ゴールをキャンセルして終了
                    return;
                }
                break;
            }
            case RobotBehaviors::WAYPOINT_NAV_PLANNING_ABORTED:
            {
                ROS_INFO("!! WAYPOINT_NAV_PLANNING_ABORTED !!");
                this->cancelGoal();          // 今のゴールをキャンセルして
                target_waypoint_index_ -= 1; // waypoint indexを１つ戻す
                break;
            }

            case RobotBehaviors::WAYPOINT_REACHED_GOAL_WAIT:
            {
                if (this->isFinalGoal())
                {                       // そのwaypointが最後だったら
                    this->cancelGoal(); // ゴールをキャンセルして終了
                    return;
                }

                ROS_INFO("[[ WAYPOINT_REACHED_GOAL_WAIT ]]");
                while (ros::ok())
                {
                    // resume（再開）フラグが来るまで待機です。
                    if (resume_) {
                        resume_ = false;    // フラグを戻す
                        break;
                    }
                    rate_.sleep();
                    ros::spinOnce();
                }
                break;
            }
            }

            rate_.sleep();
            ros::spinOnce();
        }
    }

  private:
    MoveBaseClient ac_;
    RobotBehaviors::State robot_behavior_state_; // ロボットの状態
    int target_waypoint_index_;                  // 次に目指すウェイポイントのインデックス。今目指しているWaypointではないので注意。
    ros::Rate rate_;
    std::vector<WayPoint> waypoints_; // 読み込んだWaypointの配列
    ros::Publisher next_waypoint_marker_pub_;
    ros::NodeHandle nh_;
    double reach_threshold_;       // 今セットされてるゴールへのしきい値
    geometry_msgs::Pose now_goal_; // 現在目指しているゴールの座標
    tf::TransformListener listener_;
    bool resume_;                // trueになると停止からの再開を意味する
    ros::Subscriber resume_sub_; // 一時停止からの再開指示を受け取る購読者
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_navigator");
    WaypointNavigator waypoint_navigator;
    waypoint_navigator.run();
    ROS_INFO("@@@@@@@@@@@@@@@@@@ E N D  @@@@@@@@@@@@@@@@@@");
    return 0;
}