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
    update_loop_rate_ = 10;
    data_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/3D_markers_visualization/markers", 1);

    std::string model_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/model_params.yaml";
    nh.param<std::string>("/dataloader/model_config", model_config_file, model_config_file);
    model_loader_ = new ModelLoader(model_config_file);
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

    /* updateObjectData<mas_perception_msgs::Person>();              // has no name field */
    updateObjectData<mas_perception_msgs::Object>();
    updateObjectData<mas_perception_msgs::Plane>();               // has no pose field (only position in plane_point)

    // Remove objects from map if not database
    std::cout << "Items to be deleted:" << std::endl;

    /* for (auto &item_to_delete_name : item_delete_list_) */
    /* { */
    /*     std::cout << item_to_delete_name << std::endl; */
    /*     object_data_.erase(item_to_delete_name); */
    /* } */

    for (auto &delete_list : item_delete_map_)
    {
        auto delete_item_type = delete_list.first;
        for (auto &item_to_delete_name : item_delete_map_[delete_item_type])
        {
            std::cout << item_to_delete_name << std::endl;
            object_data_record_[delete_item_type].erase(item_to_delete_name);
        }
    }

    std::cout << "\nDetails of currently stored objects:" << std::endl;
    printStoredObjectData();
}

void MDRDataloader::runDataUpdateLoop()
{
    ROS_INFO("[mdr_dataloader] Starting data update loop...");
    ros::Rate rate(update_loop_rate_);

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
    for (auto it = object_data_.begin(); it != object_data_.end(); it++)
    {
        std::cout << "Entry " << entry_counter++ << ":" << std::endl;
        std::cout << "ID: " << it->first << std::endl;
    }
}

void MDRDataloader::publishObjectData()
{
    visualization_msgs::MarkerArray marker_array_msg;

    /* for (auto &object : object_data_) */
    for (auto &object_data : object_data_record_)
    {
        for (auto &object : object_data.second)
        {
            MeshData *mesh_data = dynamic_cast<MeshData*>(object.second);
            if (mesh_data)
            {
                auto marker = model_loader_->getMeshMarker(mesh_data->unique_id_, mesh_data->type_, "base_link", "", 
                                                           mesh_data->pose_);
                std::cout << "\nAdding MeshData object marker:" << std::endl;
                std::cout << "Unique ID: " << mesh_data->unique_id_ << std::endl;
                std::cout << "Type: " << mesh_data->type_ << std::endl;
                std::cout << "Pose x: " << mesh_data->pose_ << std::endl;
                marker_array_msg.markers.push_back(*marker);
                continue;
            }

            PlaneData *plane_data = dynamic_cast<PlaneData*>(object.second);
            if (plane_data)
            {
                auto marker = model_loader_->getPlaneMarker(plane_data->unique_id_, "base_link", "", 
                                                            plane_data->center_, plane_data->convex_hull_);
                std::cout << "\nAdding PlaneData object marker:" << std::endl;
                std::cout << "Unique ID: " << plane_data->unique_id_ << std::endl;
                std::cout << "Center x: " << plane_data->center_.x() << std::endl;
                marker_array_msg.markers.push_back(*marker);
                continue;
            }
        }
    }

    for (auto &marker_id : marker_delete_list_)
    {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "base_link";
        delete_marker.id = marker_id;
        marker_array_msg.markers.push_back(delete_marker);
    }

    data_pub_.publish(marker_array_msg);
}
