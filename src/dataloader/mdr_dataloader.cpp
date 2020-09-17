/**
     File: mdr_dataloader.cpp
     Purpose: ...
     @author Ahmed Faisal Abdelrahman
     @author Sushant Vijay Chavan
     @version 1.0
 */


#include "dataloader/mdr_dataloader.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace RVizDataLoader;

MDRDataloader::MDRDataloader(ros::NodeHandle nh) : AbstractDataloader(nh) 
{
    update_loop_rate_ = 10;
    data_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rviz_3d_object_visualizer/markers", 1);

    std::string model_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/model_params.yaml";
    obj_category_mesh_filepath_ = ros::package::getPath("rviz_3d_object_visualizer") + "/config/object_mesh_categories.yaml";
    nh.param<std::string>("/dataloader/model_config", model_config_file, model_config_file);
    model_loader_ = new ModelLoader(model_config_file);

    fillObjectCategoryMeshMap();
}

MDRDataloader::~MDRDataloader()
{
}

void MDRDataloader::fillObjectCategoryMeshMap()
{
    YAML::Node yaml_node = YAML::LoadFile(obj_category_mesh_filepath_);
    for (const auto& entry: yaml_node)
        obj_category_mesh_map_.insert(std::pair<std::string, Mesh::Types>(entry.first.as<std::string>(), Mesh::getMeshType(entry.second.as<std::string>())));
}

Mesh::Types MDRDataloader::getObjectMeshType(std::string object_category)
{
    if(obj_category_mesh_map_.count(object_category) != 0)
        return obj_category_mesh_map_[object_category];
    else
        return Mesh::Types::UNKNOWN;
}

void MDRDataloader::queryDatabase()
{
    std::cout << "\nDetails of new objects in database:" << std::endl;

    updateObjectData<mas_perception_msgs::Person>();              // has no name field in old message type
    updateObjectData<mas_perception_msgs::Object>();
    updateObjectData<mas_perception_msgs::Plane>();

    // Remove objects from map if not database
    std::cout << "\nItems to be deleted:" << std::endl;

    for (auto &delete_list : item_delete_map_)
    {
        auto delete_item_type = delete_list.first;
        for (auto &item_to_delete_name : item_delete_map_[delete_item_type])
        {
            std::cout << item_to_delete_name << std::endl;
            object_data_record_[delete_item_type].erase(item_to_delete_name);
        }
        item_delete_map_[delete_item_type].clear();
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
    for (auto &object_data : object_data_record_)
    {
        for (auto it = object_data.second.begin(); it != object_data.second.end(); it++)
        {
            std::cout << "Entry " << entry_counter++ << ":" << std::endl;
            std::cout << "ID: " << it->first << std::endl;
        }
    }
}

void MDRDataloader::publishObjectData()
{
    visualization_msgs::MarkerArray marker_array_msg;

    for (auto &object_data : object_data_record_)
    {
        for (auto &object : object_data.second)
        {
            MeshData *mesh_data = dynamic_cast<MeshData*>(object.second);
            if (mesh_data)
            {
                std::string mesh_name;
                if (mesh_data->type_ == Mesh::Types::PERSON)
                    mesh_name = Mesh::MeshTypesMap.at(mesh_data->type_) + "/" + mesh_data->name_;
                else
                    mesh_name = "OBJECT/" + Mesh::MeshTypesMap.at(mesh_data->type_) + "/" + mesh_data->name_;

                auto marker = model_loader_->getMeshMarker(mesh_data->unique_id_, mesh_data->type_, mesh_name, "base_link", "", 
                                                           mesh_data->pose_);
                if (marker.first)
                {
                    marker.first->action = visualization_msgs::Marker::ADD;
                    marker_array_msg.markers.push_back(*(marker.first));
                    marker.second->action = visualization_msgs::Marker::ADD;
                    marker_array_msg.markers.push_back(*(marker.second));
                }
                continue;
            }

            PlaneData *plane_data = dynamic_cast<PlaneData*>(object.second);
            if (plane_data)
            {
                std::string plane_name = "PLANE/" + plane_data->name_;
                auto marker = model_loader_->getPlaneMarker(plane_data->unique_id_, plane_name, "base_link", "", 
                                                            plane_data->center_, plane_data->convex_hull_);
                if (marker.first)
                {
                    marker.first->action = visualization_msgs::Marker::ADD;
                    marker_array_msg.markers.push_back(*(marker.first));
                    marker.second->action = visualization_msgs::Marker::ADD;
                    marker_array_msg.markers.push_back(*(marker.second));
                }
                continue;
            }
        }
    }

    for (auto &marker_id : marker_delete_list_)
    {
        visualization_msgs::Marker delete_marker;
        visualization_msgs::Marker text_delete_marker;

        delete_marker.header.frame_id = "base_link";
        delete_marker.id = marker_id;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        marker_array_msg.markers.push_back(delete_marker);

        text_delete_marker.header.frame_id = "base_link";
        text_delete_marker.id = marker_id + 1;
        text_delete_marker.action = visualization_msgs::Marker::DELETE;
        marker_array_msg.markers.push_back(text_delete_marker);
    }

    data_pub_.publish(marker_array_msg);
}
