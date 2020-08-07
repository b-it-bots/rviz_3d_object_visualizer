#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "dataloader/model_loader.h"

int main(int argc, char** argv)
{
    RVizDataLoader::ModelLoader model_loader = RVizDataLoader::ModelLoader("/home/suvich15/catkin/domestic_robotics/src/extras/rviz_3d_object_visualizer/config/model_params.yaml");
    auto bottle_marker = model_loader.getMarker(1, RVizDataLoader::Model::Types::BOTTLE, "base_link", "");
    bottle_marker->pose.position.z += 0.91;
    auto table_marker = model_loader.getMarker(2, RVizDataLoader::Model::Types::TABLE, "base_link", "");
    auto chair_marker = model_loader.getMarker(3, RVizDataLoader::Model::Types::CHAIR, "base_link", "");
    chair_marker->pose.position.x -= 0.25;
    auto person_marker = model_loader.getMarker(4, RVizDataLoader::Model::Types::PERSON, "base_link", "");
    person_marker->pose.position.x += 1.0;

    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher bottle_pub = n.advertise< visualization_msgs::Marker >("visualization_marker/bottle", 1);
    ros::Publisher table_pub = n.advertise< visualization_msgs::Marker >("visualization_marker/table", 1);
    ros::Publisher chair_pub = n.advertise< visualization_msgs::Marker >("visualization_marker/chair", 1);
    ros::Publisher person_pub = n.advertise< visualization_msgs::Marker >("visualization_marker/person", 1);

    while (ros::ok()) 
    {
        // Publish the marker
        bottle_pub.publish(*bottle_marker);
        table_pub.publish(*table_marker);
        chair_pub.publish(*chair_marker);
        person_pub.publish(*person_marker);

        r.sleep();
    }
}
