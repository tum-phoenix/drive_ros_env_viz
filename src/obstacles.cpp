#include "ros/ros.h"
#include "drive_ros_msgs/ObstacleArray.h"
#include "visualization_msgs/MarkerArray.h"

ros::Publisher pub_marker;
std::string out_ns;

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

    for(auto it = msg->obstacles.begin(); it != msg->obstacles.end(); it++)
    {
        // draw points
        visualization_msgs::Marker ob;
        for(auto pt = it->polygon.points.begin(); pt != it->polygon.points.end(); pt++)
        {
            geometry_msgs::Point gpt;
            gpt.x = pt->x;
            gpt.y = pt->y;
            gpt.z = pt->z;
            ob.points.push_back(gpt);
        }

        ob.header.stamp = it->header.stamp;
        ob.header.frame_id = it->header.frame_id;
        ob.ns = out_ns;
        ob.id = id;
        ob.type = visualization_msgs::Marker::POINTS;
        ob.action = visualization_msgs::Marker::ADD;
        ob.scale.x = 0.01; // width
        ob.scale.y = 0.01; // height
        ob.color.a = it->trust;
        ob.color.r = 0.0;
        ob.color.g = 1.0;
        ob.color.b = 0.0;

        id++;
        obs.markers.push_back(ob);

        // draw centroid
        visualization_msgs::Marker ct;
        ct.header.stamp = it->header.stamp;
        ct.header.frame_id = it->header.frame_id;
        ct.ns = out_ns;
        ct.id = id;
        ct.type = visualization_msgs::Marker::SPHERE;
        ct.action = visualization_msgs::Marker::ADD;
        ct.pose.position = it->centroid;
        ct.scale.x = 0.05;
        ct.scale.y = 0.05;
        ct.scale.z = 0.05;
        ct.color.a = it->trust;
        ct.color.r = 1.0;
        ct.color.g = 0.0;
        ct.color.b = 0.0;
        id++;
        obs.markers.push_back(ct);
    }
    pub_marker.publish(obs);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacles");
    ros::NodeHandle nh;

    nh.param<std::string>("out_ns", out_ns, "roadLineViz");

    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("marker_out", 1);
    ros::Subscriber sub = nh.subscribe("obstacles_in", 1, cb);

    ros::spin();

    return 0;
}
