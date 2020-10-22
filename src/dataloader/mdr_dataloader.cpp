/*
 * MIT License
 * 
 * Copyright (c) 2020 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "dataloader/mdr_dataloader.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace RVizDataLoader;

MDRDataloader::MDRDataloader(ros::NodeHandle nh) : AbstractDataloader(nh) 
{
    ros::param::get("~update_loop_rate", update_loop_rate_);
    ros::param::get("~marker_pub_topic", marker_pub_topic_);
    ros::param::get("~obj_category_mesh_filename", obj_category_mesh_filename_);
    ros::param::get("~model_config_filename", model_config_filename_);
    ros::param::get("~debug", debug_);

    data_pub_ = nh.advertise<visualization_msgs::MarkerArray>(marker_pub_topic_, 1);
    model_loader_ = new ModelLoader(ros::package::getPath("rviz_3d_object_visualizer") + "/config/" + model_config_filename_);

    fillObjectCategoryMeshMap();
    ROS_INFO("[mdr_dataloader] Initialized.");
}

MDRDataloader::~MDRDataloader()
{
    delete model_loader_;
}

void MDRDataloader::fillObjectCategoryMeshMap()
{
    YAML::Node yaml_node = YAML::LoadFile(ros::package::getPath("rviz_3d_object_visualizer") + "/config/" + obj_category_mesh_filename_);
    for (const auto& entry: yaml_node)
        obj_category_mesh_map_.insert(std::pair<std::string, Mesh::Types>(entry.first.as<std::string>(), Mesh::getMeshType(entry.second.as<std::string>())));
}

Mesh::Types MDRDataloader::getObjectMeshType(std::string object_category)
{
    if(obj_category_mesh_map_.count(object_category))
        return obj_category_mesh_map_[object_category];
    else
        return Mesh::Types::UNKNOWN;
}

void MDRDataloader::queryDatabase()
{
    if (debug_)
    {
        ROS_INFO("[mdr_dataloader] Querying MongoDB database...");
        ROS_INFO("[mdr_dataloader] Displaying details of objects found in database:");
    }

    updateObjectData<mas_perception_msgs::Person>();
    updateObjectData<mas_perception_msgs::Object>();
    updateObjectData<mas_perception_msgs::Plane>();

    // Remove objects from map if not database
    for (auto &delete_list : item_delete_map_)
    {
        auto delete_item_type = delete_list.first;
        for (auto &item_to_delete_name : item_delete_map_[delete_item_type])
            object_data_record_[delete_item_type].erase(item_to_delete_name);
        item_delete_map_[delete_item_type].clear();
    }

    if (debug_)
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
    ROS_INFO("[mdr_dataloader] Displaying details of objects currently stored in the database:");
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
    marker_delete_list_.clear();

    data_pub_.publish(marker_array_msg);
}
