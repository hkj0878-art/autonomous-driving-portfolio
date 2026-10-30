// 복원본: 2025-09~10 기록을 참고한 신규 구현. 원본 실차 코드가 아님.
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <algorithm>
#include <string>

class LocalPathSelector {
public:
  LocalPathSelector() : nh_(), pnh_("~") {
    global_sub_ = nh_.subscribe("/global_path", 1, &LocalPathSelector::globalCb, this);
    lane_sub_ = nh_.subscribe("/lane_path", 1, &LocalPathSelector::laneCb, this);
    uturn_sub_ = nh_.subscribe("/uturn_path", 1, &LocalPathSelector::uturnCb, this);
    avoidance_sub_ = nh_.subscribe("/avoidance_path", 1, &LocalPathSelector::avoidanceCb, this);
    parking_sub_ = nh_.subscribe("/parking_path", 1, &LocalPathSelector::parkingCb, this);
    odom_sub_ = nh_.subscribe("/gps_utm_odom", 1, &LocalPathSelector::odomCb, this);
    region_sub_ = nh_.subscribe("/region", 1, &LocalPathSelector::regionCb, this);
    out_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    selected_out_ = nh_.advertise<nav_msgs::Path>("/selected_path", 1);
    path_num_pub_ = nh_.advertise<std_msgs::Int32>("/parking_path_number", 1, true);
    gear_pub_ = nh_.advertise<std_msgs::String>("/parking_gear", 1, true);
    pnh_.param("end_threshold_m", end_thresh_, 1.0);
    pnh_.param("uturn_leave_distance_m", uturn_leave_, 2.0);
    pnh_.param("uturn_rejoin_distance_m", uturn_rejoin_, 2.0);
    pnh_.param("avoidance_distance_m", avoidance_dist_, 6.0);
    timer_ = nh_.createTimer(ros::Duration(0.1), &LocalPathSelector::tick, this);
  }
private:
  static bool valid(const nav_msgs::Path &p) { return !p.poses.empty(); }
  static double d(double x1, double y1, double x2, double y2) { return std::hypot(x1-x2,y1-y2); }
  static double distanceToEnd(const nav_msgs::Path &p, const nav_msgs::Odometry &o) {
    if (!valid(p)) return INFINITY;
    const auto &a = p.poses.back().pose.position;
    const auto &b = o.pose.pose.position;
    return d(a.x,a.y,b.x,b.y);
  }
  double nearest(const nav_msgs::Path &p) const {
    if (!valid(p) || !have_odom_) return INFINITY;
    const auto &q = odom_.pose.pose.position;
    double best = INFINITY;
    for (const auto &pt : p.poses) {
      const auto &a = pt.pose.position;
      best = std::min(best,d(q.x,q.y,a.x,a.y));
    }
    return best;
  }
  void globalCb(const nav_msgs::Path::ConstPtr& m){global_=*m; global_update_=ros::Time::now();}
  void laneCb(const nav_msgs::Path::ConstPtr& m){lane_=*m;}
  void uturnCb(const nav_msgs::Path::ConstPtr& m){uturn_=*m;}
  void avoidanceCb(const nav_msgs::Path::ConstPtr& m){avoid_=*m;}
  void parkingCb(const nav_msgs::Path::ConstPtr& m){parking_=*m;}
  void odomCb(const nav_msgs::Odometry::ConstPtr& m){odom_=*m;have_odom_=true;}
  void regionCb(const std_msgs::String::ConstPtr& m){
    if (region_ != m->data) {
      if (m->data == "uturn") {uturn_started_=false; uturn_exiting_=false;}
      if (m->data == "stop") {stop_requested_=false; stop_done_=false; awaiting_new_global_=false;}
    }
    region_=m->data;
  }
  void publishNum(int n){std_msgs::Int32 msg;msg.data=n;path_num_pub_.publish(msg);}
  void publishGear(const std::string &gear){std_msgs::String msg;msg.data=gear;gear_pub_.publish(msg);}
  void tick(const ros::TimerEvent&) {
    if (!have_odom_) return;
    const nav_msgs::Path *selection=nullptr;
    const bool is_parking = region_=="parking_1"||region_=="parking_2"||
      region_=="parking_3"||region_=="parking_4";
    if (is_parking) {
      int num=region_.back()-'0';
      parking_number_=num;
      selection=valid(parking_)?&parking_:&global_;
    } else if (region_=="stop") {
      if (!stop_requested_) {
        publishNum(parking_number_+4);
        stop_requested_=true;
        // 새로운 /global_path를 수신한 후에만 주차용 출력 허용
        request_time_=ros::Time::now();
      }
      if (!stop_done_) {
        if (global_update_ <= request_time_) return;
        selection=valid(global_)?&global_:nullptr;
        if (selection && distanceToEnd(*selection,odom_)<=end_thresh_) {
          publishNum(0); publishGear("D"); stop_done_=true;
          awaiting_new_global_=true; request_time_=ros::Time::now();
          selection=nullptr;
        }
      } else {
        if (awaiting_new_global_ && global_update_<=request_time_) return;
        awaiting_new_global_=false;
        selection=valid(global_)?&global_:nullptr;
      }
    } else if (region_=="uturn" && valid(uturn_)) {
      if (!uturn_started_) {uturn_started_=true; uturn_exiting_=false;}
      if (!uturn_exiting_) {
        selection=&uturn_;
        if (distanceToEnd(uturn_, odom_)<=uturn_leave_) uturn_exiting_=true;
      } else {
        if (valid(global_) && nearest(global_)<=uturn_rejoin_)
          selection=&global_;
        else
          selection=&uturn_;
      }
    } else {
      // 회피경로와 차량 간 최소거리만으로 접근 예시. 실제 장애물 기반 검증과 다름.
      if (valid(avoid_) && nearest(avoid_)<=avoidance_dist_) selection=&avoid_;
      else if (valid(lane_)) selection=&lane_;
      else selection=&global_;
    }
    if (!selection || !valid(*selection)) {ROS_WARN_THROTTLE(2.0,"empty local path, skip publish"); return;}
    // 동일한 map frame을 요구. tf 누락된 혼합 map/base_link 경로를 자동 사용하지 않음.
    if (!odom_.header.frame_id.empty() && !selection->header.frame_id.empty() &&
        odom_.header.frame_id!=selection->header.frame_id) {
      ROS_ERROR_THROTTLE(2.0,"frame mismatch: odom=%s path=%s", odom_.header.frame_id.c_str(),selection->header.frame_id.c_str());
      return;
    }
    nav_msgs::Path msg=*selection;
    msg.header.stamp=ros::Time::now();
    out_.publish(msg);
    selected_out_.publish(msg);
  }
  ros::NodeHandle nh_,pnh_;
  ros::Subscriber global_sub_,lane_sub_,uturn_sub_,avoidance_sub_,parking_sub_,odom_sub_,region_sub_;
  ros::Publisher out_,selected_out_,path_num_pub_,gear_pub_;
  ros::Timer timer_;
  nav_msgs::Path global_,lane_,uturn_,avoid_,parking_;
  nav_msgs::Odometry odom_;
  std::string region_="go";
  ros::Time global_update_,request_time_;
  bool have_odom_=false,stop_requested_=false,stop_done_=false,awaiting_new_global_=false;
  bool uturn_started_=false,uturn_exiting_=false;
  int parking_number_=1;
  double end_thresh_=1.0,uturn_leave_=2.0,uturn_rejoin_=2.0,avoidance_dist_=6.0;
};
int main(int argc,char** argv){ros::init(argc,argv,"localpath_pub_recovered");LocalPathSelector x;ros::spin();return 0;}
