// ROS1 educational reimplementation, NOT the original cluster-based parking algorithm.
// Goal provider deliberately separate: /parking_targets PoseArray = [mid, goal, goal2] in map frame.
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

class ParkingPlanner {
public:
  ParkingPlanner() : nh_() {
    target_sub_=nh_.subscribe("/parking_targets",1,&ParkingPlanner::targetsCb,this);
    pose_sub_=nh_.subscribe("/gps_utm_odom",1,&ParkingPlanner::poseCb,this);
    path_pub_=nh_.advertise<nav_msgs::Path>("/parking_path",1,true);
    gear_pub_=nh_.advertise<std_msgs::String>("/parking_gear",1,true);
    ROS_WARN("RECONSTRUCTION: requires /parking_targets, not recovered cluster-to-slot planner");
  }
private:
  static double dist(const geometry_msgs::Point&a,const geometry_msgs::Point&b) {
    return std::hypot(a.x-b.x,a.y-b.y);
  }
  geometry_msgs::PoseStamped point(const geometry_msgs::Point&p,const std::string& frame) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id=frame;pose.pose.position=p;pose.pose.orientation.w=1.;return pose;
  }
  static geometry_msgs::Point lerp(const geometry_msgs::Point&a,const geometry_msgs::Point&b,double t) {
    geometry_msgs::Point p;
    p.x=a.x+(b.x-a.x)*t;p.y=a.y+(b.y-a.y)*t;p.z=a.z+(b.z-a.z)*t;return p;
  }
  void addSegment(nav_msgs::Path& out,const geometry_msgs::Point&a,const geometry_msgs::Point&b,int n) {
    for(int i=0;i<n;++i) out.poses.push_back(point(lerp(a,b,double(i)/double(n-1)),out.header.frame_id));
  }
  void targetsCb(const geometry_msgs::PoseArray::ConstPtr& msg){
    if(initialized_ || !have_pose_ || msg->poses.size()<3) return;
    if(!msg->header.frame_id.empty() && msg->header.frame_id!="map") {
      ROS_ERROR("parking targets must be in map frame");return;
    }
    mid_=msg->poses[0].position;goal_=msg->poses[1].position;goal2_=msg->poses[2].position;
    nav_msgs::Path path;path.header.frame_id="map";path.header.stamp=ros::Time::now();
    geometry_msgs::Point start=ego_.pose.pose.position;
    // approach: mid 목표까지 160점 cubic Bézier, 제어점은 시작·종료 헤딩 이용
    double len=std::max(0.3,std::min(2.0,dist(start,mid_)/3.0));
    double syaw=tf2::getYaw(ego_.pose.pose.orientation),myaw=tf2::getYaw(msg->poses[0].orientation);
    geometry_msgs::Point p1=start,p2=mid_;
    p1.x+=len*std::cos(syaw);p1.y+=len*std::sin(syaw);
    p2.x-=len*std::cos(myaw);p2.y-=len*std::sin(myaw);
    for(int i=0;i<160;++i){
      double t=double(i)/159.,s=1.-t;
      geometry_msgs::Point p;
      p.x=s*s*s*start.x+3*s*s*t*p1.x+3*s*t*t*p2.x+t*t*t*mid_.x;
      p.y=s*s*s*start.y+3*s*s*t*p1.y+3*s*t*t*p2.y+t*t*t*mid_.y;
      path.poses.push_back(point(p,"map"));
    }
    addSegment(path,mid_,goal_,40);
    addSegment(path,goal_,goal2_,40);
    initialized_=true;path_pub_.publish(path);publishGear("D");
    ROS_INFO("parking goals locked, path poses=%zu",path.poses.size());
  }
  void poseCb(const nav_msgs::Odometry::ConstPtr&m){
    if(m->header.frame_id!="map") {ROS_WARN_THROTTLE(5.,"expecting odom frame map");return;}
    ego_=*m;have_pose_=true;
    if(!initialized_) return;
    const auto &pos=ego_.pose.pose.position;
    if(stage_==0 && dist(pos,mid_)<=0.4){stage_=1;publishGear("R");}
    else if(stage_==1 && dist(pos,goal_)<=0.4){stage_=2;publishGear("D");}
    else if(stage_==2 && dist(pos,goal2_)<=0.4){stage_=3;publishGear("P");}
  }
  void publishGear(const std::string& s){std_msgs::String m;m.data=s;gear_pub_.publish(m);}
  ros::NodeHandle nh_;
  ros::Subscriber target_sub_,pose_sub_;
  ros::Publisher path_pub_,gear_pub_;
  nav_msgs::Odometry ego_;
  geometry_msgs::Point mid_,goal_,goal2_;
  bool have_pose_=false,initialized_=false;
  int stage_=0;
};
int main(int argc,char**argv){ros::init(argc,argv,"parking_planner_recovered");ParkingPlanner p;ros::spin();return 0;}
