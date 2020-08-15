/**
     File: mdr_dataloader.cpp
     Purpose: ...
     @author Ahmed Faisal Abdelrahman
     @author Sushant Vijay Chavan
     @version 1.0
 */


#include "dataloader/mdr_dataloader.h"
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace RVizDataLoader;

MDRDataloader::MDRDataloader(ros::NodeHandle nh) : AbstractDataloader(nh) 
{
    update_loop_rate = 10;
    object_data_pub= nh.advertise<visualization_msgs::MarkerArray>("/3D_markers_visualization/markers", 1);

    std::string model_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/model_params.yaml";
    nh.param<std::string>("/dataloader/model_config", model_config_file, model_config_file);
    model_loader = new ModelLoader(model_config_file);
}

MDRDataloader::~MDRDataloader()
{
}

void MDRDataloader::queryDatabase()
{
    // Adding new entries:
    /* mas_perception_msgs::Object object_msg; */
    /* mas_perception_msgs::Person person_msg; */
    /* mas_perception_msgs::Plane plane_msg; */
    /* message_proxy_.insertNamed("Object_msg", object_msg); */
    /* message_proxy_.insertNamed("Person_msg", person_msg); */
    /* message_proxy_.insertNamed("Plane_msg", plane_msg); */

    std::cout << "\nDetails of new objects in database:" << std::endl;

    /* updateObjectData<mas_perception_msgs::Person>(); */
    updateObjectData<mas_perception_msgs::Object>();
    updateObjectData<mas_perception_msgs::Plane>();

    // Remove objects from map if not database
    std::cout << "Items to be deleted:" << std::endl;
    for (auto &item_to_delete_name : item_delete_list_)
    {
        std::cout << item_to_delete_name << std::endl;
        object_data.erase(item_to_delete_name);
    }

    std::cout << "\nDetails of currently stored objects:" << std::endl;
    printStoredObjectData();
}

void MDRDataloader::runDataUpdateLoop()
{
    ROS_INFO("[mdr_dataloader] Starting data update loop...");
    ros::Rate rate(update_loop_rate);

    while (ros::ok())
    {
        queryDatabase();
        publishObjectData();

        ros::spinOnce();
        rate.sleep();
    }
}

void MDRDataloader::printStoredObjectData()
{
    int entry_counter{0};
    for (auto it = object_data.begin(); it != object_data.end(); it++)
    {
        std::cout << "Entry " << entry_counter++ << ":" << std::endl;
        std::cout << "ID: " << it->first << std::endl;
    }
}

void MDRDataloader::publishObjectData()
{
    // TODO:
 
    visualization_msgs::MarkerArray msg;

    for (auto &object : object_data)
    {
        auto marker = model_loader->getMarker(object.second.unique_id_, object.second.type_, "base_link", "", 
                                              object.second.pose_);
        msg.markers.push_back(*marker);
    }

    object_data_pub.publish(msg);
}
