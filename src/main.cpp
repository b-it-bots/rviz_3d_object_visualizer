#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "dataloader/modelloader/model_loader.h"

using namespace RVizDataLoader;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    std::string model_config_file;
    n.param<std::string>("/dataloader/model_config", model_config_file, 
                         "/home/suvich15/catkin/domestic_robotics/src/extras/rviz_3d_object_visualizer/config/model_params.yaml");
    if (model_config_file == "")
    {
        std::cout << "No model config path set! Please set the path in the rosparam \"/dataloader/model_config\"" << std::endl;
        return 1;
    }

    ModelLoader model_loader = ModelLoader(model_config_file);
    auto bottle_marker = model_loader.getMarker(1, Model::Types::BOTTLE, "base_link", "", 
                                                Utils::Pose<double>(0, 0, 0.91, 0, 0, 0));
    auto table_marker = model_loader.getMarker(2, Model::Types::TABLE, "base_link", "");
    auto chair_marker = model_loader.getMarker(3, Model::Types::CHAIR, "base_link", "", 
                                                Utils::Pose<double>(-0.25, 0, 0, 0, 0, 0));
    auto person_marker = model_loader.getMarker(4, Model::Types::PERSON, "base_link", "", 
                                                Utils::Pose<double>(1.0, 0, 0, 0, 0, 3.14));

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
