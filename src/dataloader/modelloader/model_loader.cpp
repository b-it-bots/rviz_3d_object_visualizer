#include <iostream>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "dataloader/modelloader/model_loader.h"

using namespace RVizDataLoader;

ModelLoader::ModelLoader(const std::string& model_config_path)
: yaml_loader_(model_config_path)
{
}

auto ModelLoader::loadModel(Mesh::Types mesh_type) 
                            -> std::unique_ptr<visualization_msgs::Marker>
{
    std::unique_ptr<visualization_msgs::Marker> marker = std::unique_ptr<visualization_msgs::Marker>(new visualization_msgs::Marker);

    MeshData mesh_data = yaml_loader_.getMeshConfig(mesh_type);

    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->pose.position.x = mesh_data.pose_.position.x();
    marker->pose.position.y = mesh_data.pose_.position.y();
    marker->pose.position.z = mesh_data.pose_.position.z();
    marker->pose.orientation = tf2::toMsg(Utils::toTf2Quaternion(mesh_data.pose_.orientation));

    marker->scale.x = mesh_data.scale_.x();
    marker->scale.y = mesh_data.scale_.y();
    marker->scale.z = mesh_data.scale_.z();

    if (mesh_data.use_color_from_mesh_)
    {
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 0.0;
        marker->color.a = 0.0;
        marker->mesh_use_embedded_materials = true;
    }
    else
    {
        marker->color.r = mesh_data.color_.r() / 255.0;
        marker->color.g = mesh_data.color_.g() / 255.0;
        marker->color.b = mesh_data.color_.b() / 255.0;
        marker->color.a = 1.0;
        marker->mesh_use_embedded_materials = false;
    }

    marker->mesh_resource = mesh_data.mesh_resource_;

    return std::move(marker);
}

auto ModelLoader::getMarker(int id, Mesh::Types type, 
                            const std::string& frame_id, 
                            const std::string& ns,
                            const Utils::Pose<double>& pose)
                            -> std::unique_ptr<visualization_msgs::Marker>
{
    std::unique_ptr<visualization_msgs::Marker> marker = loadModel(type);
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = ns;
    marker->id = id;

    // Translate the model in world space
    marker->pose.position.x += pose.position.x();
    marker->pose.position.y += pose.position.y();
    marker->pose.position.z += pose.position.z();

    // Orient the model in world space
    tf2::Quaternion quat_model_space;
    tf2::fromMsg(marker->pose.orientation, quat_model_space);
    tf2::Quaternion quat_world_space = Utils::toTf2Quaternion(pose.orientation) * quat_model_space;
    quat_world_space.normalize();
    marker->pose.orientation = tf2::toMsg(quat_world_space);

    return std::move(marker);
}
