
#include <ros/ros.h>
// ROSメインヘッダーファイル
// ROSプログラミングを行う際に必要となるROSファイルのインクルードを行う。
//後述するROS_INFO関数などを使用できるようになる。

#include <geometry_msgs/PoseWithCovarianceStamped.h>    // 現在位置を取得するのに使う

// #include <geometry_msgs/PointStamped.h>     // TODO:どっかに含まれてる？不要なら除去

#include <interactive_markers/interactive_marker_server.h>  // RVizのマーカーを使う

#include <tf/tf.h>

#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>             // RVizに表示するマーカー。配列バージョン

#include <turtlebot3_momonga/WaypointArray.h>

#include <math.h>
#include <string>
#include <iostream>
#include <sstream>

#include <fstream>  // CSVファイルの入出力

#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>
#include <boost/program_options.hpp>    // Boost Program Options Libraryを用いるとプログラムの実行時に付けられる引数文字列について、一般的なオプションの仕組みの定義とその取得を容易に行える。



using namespace visualization_msgs;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;           // カンマで区切られた文字列から文字列を切り出すツール
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;     // RVizのマーカーを扱うため

class WaypointGenerator
{
    public:
    WaypointGenerator():
        rate_(5)    // メンバの初期化構文
    {
        ros::NodeHandle n("~"); // パラメータ
//        n.param("dist_th", dist_th_, 1.0); // distance threshold [m]

/*
        // rtabmapでlocalization_poseという名前のトピックが発行されているか確認すること。
        // これはおそらく、コントローラーで操作してWaypointを自動生成する機能。
        odom_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/localization_pose",
                                1,
                                &WaypointGenerator::addWaypoint, this);   //これは、基準座標フレームとタイムスタンプの推定ポーズを表します。
*/
        // clicked_pointはRViz上でクリックした時に発生するのかな
        clicked_sub_ = nh_.subscribe("clicked_point", 1, &WaypointGenerator::clickedPointCallback, this);

        // RVizのマーカーを送る。MarkerArrayはその名の通り、Markerが配列になっているものです。複数のマーカーを1つのpublisherで作成できます。
        reach_marker_pub_= nh_.advertise<visualization_msgs::MarkerArray>("/reach_threshold_markers", 1);

        // 追加したWaypointをパブリッシュ。(saverがCSVへ保存するのに必要)
        waypoints_pub_ = nh_.advertise<turtlebot3_momonga::WaypointArray>("/waypoints", 1);

        waypoint_box_count_ = 0;        // Waypointの数
        // トピック名前空間 "cube"上に対話型マーカーサーバーを作成する。
        server.reset( new interactive_markers::InteractiveMarkerServer("cube") );


    }

    // CSVデータをロードして、makeWaypointMakerに渡す
    void load(std::string waypoint_file)
    {
        const int rows_num = 9; // x, y, z, Qx, Qy, Qz, Qw, waypoint type, reach_threshold
        boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
        std::ifstream ifs(waypoint_file.c_str());
        std::string line;
        while(ifs.good()){
            getline(ifs, line);
            if(line.empty()){ break; }
            tokenizer tokens(line, sep);
            std::vector<double> data;
            tokenizer::iterator it = tokens.begin();
            for(; it != tokens.end() ; ++it){
                std::stringstream ss;
                double d;
                ss << *it;
                ss >> d;
                data.push_back(d);
            }
            if(data.size() != rows_num){
                ROS_ERROR("Row size is mismatch!!");
                return;
            }else{
                geometry_msgs::PoseWithCovariance new_pose;
                new_pose.pose.position.x = data[0];
                new_pose.pose.position.y = data[1];
                new_pose.pose.position.z = data[2];
                new_pose.pose.orientation.x = data[3];
                new_pose.pose.orientation.y = data[4];
                new_pose.pose.orientation.z = data[5];
                new_pose.pose.orientation.w = data[6];
                makeWaypointMarker(new_pose, (int)data[7], data[8]);
            }
        }
        ROS_INFO_STREAM(waypoint_box_count_ << "waypoints are loaded.");
    }


    // Makerを受け取り、マーカーを操作するコントロールを設定し、マーカーを設定する
    InteractiveMarkerControl& makeWaypointMarkerControl(InteractiveMarker &msg,
                                                        int waypoint_type)
    {
        InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        msg.controls.push_back(control);
        // control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
        // msg.controls.push_back(control);
        // control.independent_marker_orientation = true;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        msg.controls.push_back(control);

        Marker marker;
        marker.type = Marker::CUBE;
        marker.scale.x = msg.scale*0.5;
        marker.scale.y = msg.scale*0.5;
        marker.scale.z = msg.scale*0.5;
//        marker.color.r = 0.05 + 1.0*(float)is_searching_area;
        marker.color.r = 0.05;          // TODO:waypoint_typeに合わせて色を変えよう。
        marker.color.g = 0.80;
        marker.color.b = 0.02;
        marker.color.a = 1.0;

        control.markers.push_back(marker);
        control.always_visible = true;
        msg.controls.push_back(control);

        return msg.controls.back();

    }

    // フィードバックが到着した時に呼ばれる
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";
    
        switch ( feedback->event_type )
        {
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            {
            reach_threshold_markers_.markers[std::stoi(feedback->marker_name)].pose
                = feedback->pose;
            reach_marker_pub_.publish(reach_threshold_markers_);
            waypoints_.waypoints[std::stoi(feedback->marker_name)].pose = feedback->pose;
            break;
            }
        }
        server->applyChanges();
    }

    // WaypointのマーカーをRViz用に作成して、マーカーサーバに渡す
    void makeWaypointMarker(const geometry_msgs::PoseWithCovariance new_pose,
                            int waypoint_type, double reach_threshold)
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.pose = new_pose.pose;
        int_marker.scale = 1;

        visualization_msgs::Marker reach_marker;
        reach_marker.header.frame_id = "map";
        reach_marker.header.stamp = ros::Time();
        reach_marker.id = waypoint_box_count_;
        reach_marker.type = visualization_msgs::Marker::CYLINDER;
        reach_marker.action = visualization_msgs::Marker::ADD;
        reach_marker.pose = new_pose.pose;
        reach_marker.scale.x = reach_threshold/2.0;
        reach_marker.scale.y = reach_threshold/2.0;
        reach_marker.scale.z = 0.1;
        reach_marker.color.a = 0.7;
        reach_marker.color.r = 0.0;
        reach_marker.color.g = 0.0;
        reach_marker.color.b = 1.0;
        reach_threshold_markers_.markers.push_back(reach_marker);
        reach_marker_pub_.publish(reach_threshold_markers_);
        
        std::stringstream s;
        s << waypoint_box_count_;
        int_marker.name = s.str();
        int_marker.description = s.str();

        makeWaypointMarkerControl(int_marker, waypoint_type);

        server->insert(int_marker);
        server->setCallback(int_marker.name, boost::bind(&WaypointGenerator::processFeedback, this, _1));
        server->applyChanges();

        turtlebot3_momonga::Waypoint waypoint;
        waypoint.number = waypoint_box_count_;
        waypoint.pose = new_pose.pose;
        waypoint.waypoint_type = waypoint_type;
        waypoint.reach_tolerance = reach_threshold/2.0;
        waypoints_.waypoints.push_back(waypoint);
        
        waypoint_box_count_++;
    }

/*
    // コントローラーで操作した時に、位置情報を受け取るのに使っていると思われる。
    void addWaypoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
    {

    }
*/


    // RVizでクリックされた時のコールバックメソッド
    void clickedPointCallback(const geometry_msgs::PointStamped &point)
    {
        geometry_msgs::PoseWithCovariance pose;
        tf::pointTFToMsg(tf::Vector3( point.point.x, point.point.y, 0), pose.pose.position);
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), pose.pose.orientation);
        makeWaypointMarker(pose, 0, 3.0);
        server->applyChanges();
    }

    // タイマーで呼び出されるコールバック関数
    void publishWaypointCallback(const ros::TimerEvent&)
    {
        reach_marker_pub_.publish(reach_threshold_markers_);
        waypoints_pub_.publish(waypoints_);
        server->applyChanges();
    }

    // タイマーで呼び出されるコールバック関数、
    void tfSendTransformCallback(const ros::TimerEvent&)
    {  
        tf::Transform t;
        ros::Time time = ros::Time::now();

        for (size_t i = 0; i < waypoints_.waypoints.size(); ++i) {
            std::stringstream s;
            s << waypoints_.waypoints[i].number;
            t.setOrigin(tf::Vector3(waypoints_.waypoints[i].pose.position.x,
                                    waypoints_.waypoints[i].pose.position.y,
                                    waypoints_.waypoints[i].pose.position.z));
            t.setRotation(tf::Quaternion(waypoints_.waypoints[i].pose.orientation.x,
                                        waypoints_.waypoints[i].pose.orientation.y,
                                        waypoints_.waypoints[i].pose.orientation.z,
                                        waypoints_.waypoints[i].pose.orientation.w));
            br_.sendTransform(tf::StampedTransform(t, time, "map", s.str()));
        }
    }

    void run()
    {
        //createTimerで指定されたレートでコールバックを呼び出すタイマーを作成します。
        ros::Timer frame_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&WaypointGenerator::publishWaypointCallback, this, _1));
        ros::Timer tf_frame_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&WaypointGenerator::tfSendTransformCallback, this, _1));
        while(ros::ok())
        {
            ros::spinOnce();
            rate_.sleep();
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    // ros::Subscriber odom_sub_;  // オドメトリの購読
    ros::Subscriber clicked_sub_;   // クリックされたことを購読
    ros::Publisher reach_marker_pub_;   // Rvizに送るマーカー配列用
    ros::Publisher waypoints_pub_;      // Waypointをセーバーで保存するためにパブリッシュ必要
    turtlebot3_momonga::WaypointArray waypoints_;
    // double dist_th_;     // distance threshold [m]
    int waypoint_box_count_;            // Waypointの数
    visualization_msgs::MarkerArray reach_threshold_markers_;   // RVizに送るマーカー
    tf::TransformBroadcaster br_;       // これを使うと座標変換を簡単に配信できる
};



int main (int argc, char** argv)
{
    ROS_INFO_STREAM("main");

    // Initialize ROS
    ros::init(argc, argv, "waypoint_generator");

    WaypointGenerator generator;

    // boostを使ってコマンドライン引数を処理する
    boost::program_options::options_description desc("Options");
    desc.add_options()
        ("help", "Print help message")
        ("load", boost::program_options::value<std::string>(), "waypoint filename");

    boost::program_options::variables_map vm;

    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if( vm.count("help") ){
            std::cout << "This is waypoint generator node" << std::endl;
            std::cerr << desc << std::endl;
            return 0;
        }
        if (vm.count("load")) {
            generator.load(vm["load"].as<std::string>());
        }


    } catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    generator.run();

    return 0;
}