#include "ros/ros.h"
#include "drive_ros_msgs/RoadLine.h"
#include "visualization_msgs/Marker.h"


ros::Publisher pub_marker;
std::string out_frame_id;
std::string out_ns;


void callback(const drive_ros_msgs::RoadLineConstPtr& msg)
{
  float r=0, g=0, b=0;

  switch (msg->lineType) {

  case drive_ros_msgs::RoadLine::LEFT:
    r = 1;
    break;
  case drive_ros_msgs::RoadLine::RIGHT:
    g = 1;
    break;
  case drive_ros_msgs::RoadLine::MIDDLE:
    b = 1;
    break;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = out_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = out_ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  for(int i=0; i<msg->points.size(); i++)
  {
    marker.points.push_back(msg->points.at(i).point);
  }

  pub_marker.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_marker");
  ros::NodeHandle nh;

  nh.param<std::string>("out_frame_id", out_frame_id, "rear_axis_middle_ground");
  nh.param<std::string>("out_ns", out_ns, "roadLineViz");

  pub_marker = nh.advertise<visualization_msgs::Marker>("marker_out", 100);
  ros::Subscriber sub = nh.subscribe("road_in", 100, callback);

  ros::spin();
  return 0;
}
