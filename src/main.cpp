#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "dataloader/modelloader/model_loader.h"

using namespace RVizDataLoader;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);

    std::string model_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/model_params.yaml";
    n.param<std::string>("/dataloader/model_config", model_config_file, model_config_file);

    ModelLoader model_loader = ModelLoader(model_config_file);
    auto bottle_marker = model_loader.getMarker(1, Model::Types::BOTTLE, "base_link", "", 
                                                Utils::Pose<double>(0, 0, 0.91, 0, 0, 0));
    auto table_marker = model_loader.getMarker(2, Model::Types::TABLE, "base_link", "");
    auto chair_marker = model_loader.getMarker(3, Model::Types::CHAIR, "base_link", "", 
                                                Utils::Pose<double>(-0.25, 0, 0, 0, 0, 0));
    auto person_marker = model_loader.getMarker(4, Model::Types::PERSON, "base_link", "", 
                                                Utils::Pose<double>(1.0, 0, 0, 0, 0, 3.14));

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/3D_markers_visualization/markers", 1);
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(*bottle_marker);
    msg.markers.push_back(*table_marker);
    msg.markers.push_back(*chair_marker);
    msg.markers.push_back(*person_marker);

    while (ros::ok()) 
    {
        // Publish the markers
        pub.publish(msg);
        r.sleep();
    }
}
