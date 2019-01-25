#include "ros/ros.h"
#include "drive_ros_msgs/DrivingLine.h"
#include "visualization_msgs/Marker.h"
#include <drive_ros_trajectory_generator/polygon_msg_operations.h>


ros::Publisher pub_marker;
std::string out_frame_id;
std::string ns;
float starting_coord;


void callback(const drive_ros_msgs::DrivingLineConstPtr& msg)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = out_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
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
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  geometry_msgs::Point marker_point;
  for (float i=starting_coord; i<msg->detectionRange; i+=0.1f) {
    marker_point.x = i;
    marker_point.y = compute_polynomial_at_location(msg, i);
    marker.points.push_back(marker_point);
  }

  pub_marker.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_marker");
  ros::NodeHandle nh;

  nh.param<std::string>("out_frame_id", out_frame_id, "rear_axis_middle_ground");
  nh.param<std::string>("out_ns", ns, "roadLineViz");
  nh.param<float>("starting_coord", starting_coord, 0.5f);

  pub_marker = nh.advertise<visualization_msgs::Marker>("marker_out", 100);
  ros::Subscriber sub = nh.subscribe("road_in", 100, callback);

  ros::spin();
  return 0;
}
