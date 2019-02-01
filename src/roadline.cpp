#include "ros/ros.h"
#include "drive_ros_msgs/DrivingLine.h"
#include "visualization_msgs/Marker.h"
#include <drive_ros_environment_model/polygon_msg_operations.h>
#include <cmath>


ros::Publisher pub_marker;
std::string out_frame_id;
std::string ns;
float starting_coord;


void callback(const drive_ros_msgs::DrivingLineConstPtr& msg)
{
  visualization_msgs::Marker marker_center;
  marker_center.header.frame_id = out_frame_id;
  marker_center.header.stamp = ros::Time::now();
  marker_center.ns = ns;
  marker_center.id = 0;
  marker_center.type = visualization_msgs::Marker::LINE_STRIP;
  marker_center.action = visualization_msgs::Marker::ADD;
  marker_center.pose.position.x = 0;
  marker_center.pose.position.y = 0;
  marker_center.pose.position.z = 0;
  marker_center.pose.orientation.x = 0.0;
  marker_center.pose.orientation.y = 0.0;
  marker_center.pose.orientation.z = 0.0;
  marker_center.pose.orientation.w = 1.0;
  marker_center.scale.x = 0.01;
  marker_center.scale.y = 0.01;
  marker_center.scale.z = 0.01;
  marker_center.color.a = 1;
  marker_center.color.r = 0.0;
  marker_center.color.g = 1.0;
  marker_center.color.b = 0.0;

  visualization_msgs::Marker marker_left = marker_center;
  marker_left.color.g = 0.0;
  marker_left.color.r = 1.0;
  marker_left.id = 1;
  visualization_msgs::Marker marker_right = marker_center;
  marker_right.color.g = 0.0;
  marker_right.color.b = 1.0;
  marker_right.id = 2;

  geometry_msgs::Point marker_point;
  std::vector<float> lane_positions;
  for (float i=starting_coord; i<msg->detectionRange; i+=0.1f) {
    marker_point.x = i;
    compute_all_lanes_at_location(msg, i, lane_positions);
    if (!std::isnan(lane_positions[0])) {
      marker_point.y = lane_positions[0];
      marker_left.points.push_back(marker_point);
    }
    marker_point.y = lane_positions[1];
    marker_center.points.push_back(marker_point);
    if (!std::isnan(lane_positions[2])) {
      marker_point.y = lane_positions[2];
      marker_right.points.push_back(marker_point);
    }
  }

  pub_marker.publish(marker_center);
  pub_marker.publish(marker_left);
  pub_marker.publish(marker_right);
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
