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

    for(auto it = msg->obstacles.begin(); it != msg->obstacles.end(); it++)
    {

        // determine color
        float r=0.0, g=0.0, b=0.0;

        switch (it->obstacle_type) {
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
        ob.ns = ns;
        ob.id = id;
        ob.type = visualization_msgs::Marker::POINTS;
        ob.action = visualization_msgs::Marker::ADD;
        ob.scale.x = 0.01; // width
        ob.scale.y = 0.01; // height
        ob.color.a = it->trust;
        ob.color.r = r;
        ob.color.g = g;
        ob.color.b = b;

        id++;
        obs.markers.push_back(ob);

        // draw centroid
        visualization_msgs::Marker ct;
        ct.header.stamp = it->header.stamp;
        ct.header.frame_id = it->header.frame_id;
        ct.ns = ns;
        ct.id = id;
        ct.type = visualization_msgs::Marker::SPHERE;
        ct.action = visualization_msgs::Marker::ADD;
        ct.pose.position.x = it->centroid.x;
        ct.pose.position.y = it->centroid.y;
        ct.pose.position.z = it->centroid.z;
        ct.scale.x = 0.05;
        ct.scale.y = 0.05;
        ct.scale.z = 0.05;
        ct.color.a = it->trust;
        ct.color.r = r;
        ct.color.g = g;
        ct.color.b = b;
        id++;
        obs.markers.push_back(ct);

        // draw box
        visualization_msgs::Marker box;
        box.header.stamp = it->header.stamp;
        box.header.frame_id = it->header.frame_id;
        box.ns = ns;
        box.id = id;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;
        box.pose.position.x = it->centroid.x;
        box.pose.position.y = it->centroid.y;
        box.pose.position.z = it->centroid.z;
        box.scale.x = std::max(float(0.01), std::cos(it->yaw)*it->length + std::sin(it->yaw)*it->width);
        box.scale.y = std::max(float(0.01), std::sin(it->yaw)*it->length + std::cos(it->yaw)*it->width);
        box.scale.z = std::max(it->height, float(0.1));
        box.color.a = it->trust;
        box.color.r = r;
        box.color.g = g;
        box.color.b = b;
        id++;
        obs.markers.push_back(box);
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
