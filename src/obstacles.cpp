#include "ros/ros.h"
#include "drive_ros_msgs/ObstacleArray.h"
#include "visualization_msgs/MarkerArray.h"

ros::Publisher pub_marker;
std::string ns;

typedef drive_ros_msgs::Obstacle ObsType;

void cb(const drive_ros_msgs::ObstacleArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray obs;

    int id = 0;

    // delete all previous
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.id = id;
    id ++;
    obs.markers.push_back(del);

    // add new obstacles
    int obstacle_counter = 0;
    for(auto it: msg->obstacles)
    {
        // determine color
        float r=0.0, g=0.0, b=0.0;

        switch (it.obstacle_type) {
        case ObsType::TYPE_CAMERA:
            b = 1.0; break;      // blue
        case ObsType::TYPE_LIDAR:
            r=1.0; g=1.0; break; // yellow
        case ObsType::TYPE_GENERIC:
        default:
            g=1.0;               // green
        }

        // draw points
        visualization_msgs::Marker ob;
        for(auto pt : it.polygon.points)
        {
            geometry_msgs::Point gpt;
            gpt.x = pt.x;
            gpt.y = pt.y;
            gpt.z = pt.z;
            ob.points.push_back(gpt);
        }

        ob.header.stamp = it.header.stamp;
        ob.header.frame_id = it.header.frame_id;
        ob.ns = ns;
        ob.id = id;
        ob.type = visualization_msgs::Marker::POINTS;
        ob.action = visualization_msgs::Marker::ADD;
        ob.scale.x = 0.01; // width
        ob.scale.y = 0.01; // height
        ob.color.a = it.trust;
        ob.color.r = r;
        ob.color.g = g;
        ob.color.b = b;
        id++;
        obs.markers.push_back(ob);

        // draw centroid
        visualization_msgs::Marker ct;
        ct.header.stamp = it.header.stamp;
        ct.header.frame_id = it.header.frame_id;
        ct.ns = ns;
        ct.id = id;
        ct.type = visualization_msgs::Marker::SPHERE;
        ct.action = visualization_msgs::Marker::ADD;
        ct.pose = it.centroid_pose.pose;
        ct.scale.x = 0.05;
        ct.scale.y = 0.05;
        ct.scale.z = 0.05;
        ct.color.a = it.trust;
        ct.color.r = r;
        ct.color.g = g;
        ct.color.b = b;
        id++;
        obs.markers.push_back(ct);

        // draw box
        visualization_msgs::Marker box;
        box.header.stamp = it.header.stamp;
        box.header.frame_id = it.header.frame_id;
        box.ns = ns;
        box.id = id;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;
        box.pose = it.centroid_pose.pose;
        box.scale.x = std::max(it.length, float(0.01));
        box.scale.y = std::max(it.width,  float(0.01));
        box.scale.z = std::max(it.height, float(0.01));
        box.color.a = it.trust;
        box.color.r = r;
        box.color.g = g;
        box.color.b = b;
        id++;
        obs.markers.push_back(box);

        // draw box
        visualization_msgs::Marker text;
        text.header.stamp = it.header.stamp;
        text.header.frame_id = it.header.frame_id;
        text.ns = ns;
        text.id = id;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = it.centroid_pose.pose;
        text.scale.z = 0.1;
        text.color.a = 1.0;
        text.color.r = r;
        text.color.g = g;
        text.color.b = b;
        text.text = std::to_string(obstacle_counter);
        id++;
        obs.markers.push_back(text);

        obstacle_counter++;

    }
    pub_marker.publish(obs);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacles");
    ros::NodeHandle nh;

    nh.param<std::string>("out", ns, "obstacles");

    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("marker_out", 1);
    ros::Subscriber sub = nh.subscribe("obstacles_in", 1, cb);

    ros::spin();

    return 0;
}
