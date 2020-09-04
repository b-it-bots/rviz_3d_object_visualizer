#include <iostream>
#include <typeinfo>

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

    std::string mesh_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/model_params.yaml";
    n.param<std::string>("/dataloader/model_config", mesh_config_file, mesh_config_file);

    ModelLoader model_loader = ModelLoader(mesh_config_file);
    auto bottle_marker = model_loader.getMeshMarker(1, Mesh::Types::BOTTLE, "base_link", "", 
                                                Utils::Pose<double>(0, 0, 0.91, 0, 0, 0));
    auto table_marker = model_loader.getMeshMarker(2, Mesh::Types::TABLE, "base_link", "");
    auto chair_marker = model_loader.getMeshMarker(3, Mesh::Types::CHAIR, "base_link", "", 
                                                Utils::Pose<double>(-0.25, 0, 0, 0, 0, 0));
    auto person_marker = model_loader.getMeshMarker(4, Mesh::Types::PERSON, "base_link", "", 
                                                Utils::Pose<double>(1.0, 0, 0, 0, 0, 3.14));

    Utils::Vec3Array<double> plane_vert;
    plane_vert.push_back(Utils::Vec3<double>(-1, 1, 0.5));
    plane_vert.push_back(Utils::Vec3<double>(1, 1, 0.5));
    plane_vert.push_back(Utils::Vec3<double>(1, -1, 0.5));
    plane_vert.push_back(Utils::Vec3<double>(-1, -1, 0.5));
    auto plane_marker = model_loader.getPlaneMarker(5, "base_link", "", Utils::Vec3<double>(0, 0, 0.5), plane_vert);

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/3D_markers_visualization/markers", 1);
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(*bottle_marker);
    msg.markers.push_back(*table_marker);
    msg.markers.push_back(*chair_marker);
    msg.markers.push_back(*person_marker);
    msg.markers.push_back(*plane_marker);

    while (ros::ok()) 
    {
        // Publish the markers
        pub.publish(msg);
        r.sleep();
    }
}
