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

auto ModelLoader::loadModel(Model::Types model_type) 
                            -> std::unique_ptr<visualization_msgs::Marker>
{
    std::unique_ptr<visualization_msgs::Marker> marker = std::unique_ptr<visualization_msgs::Marker>(new visualization_msgs::Marker);

    ModelData model_data = yaml_loader_.getModelConfig(model_type);

    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->pose.position.x = model_data.pose_.position.x();
    marker->pose.position.y = model_data.pose_.position.y();
    marker->pose.position.z = model_data.pose_.position.z();

    tf2::Quaternion quat;
    quat.setRPY(model_data.pose_.orientation.roll(), 
                model_data.pose_.orientation.pitch(), 
                model_data.pose_.orientation.yaw());
    quat.normalize();
    marker->pose.orientation = tf2::toMsg(quat);

    marker->scale.x = model_data.scale_.x();
    marker->scale.y = model_data.scale_.y();
    marker->scale.z = model_data.scale_.z();

    if (model_data.use_color_from_mesh_)
    {
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 0.0;
        marker->color.a = 0.0;
        marker->mesh_use_embedded_materials = true;
    }
    else
    {
        marker->color.r = model_data.color_.r() / 255.0;
        marker->color.g = model_data.color_.g() / 255.0;
        marker->color.b = model_data.color_.b() / 255.0;
        marker->color.a = 1.0;
        marker->mesh_use_embedded_materials = false;
    }

    marker->mesh_resource = model_data.mesh_resource_;

    return std::move(marker);
}

auto ModelLoader::getMarker(int id, Model::Types type, 
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

    // Update the position of the model
    marker->pose.position.x += pose.position.x();
    marker->pose.position.y += pose.position.y();
    marker->pose.position.z += pose.position.z();

    // Update the orientation of the model
    tf2::Quaternion quat_new, quat_model;
    quat_new.setRPY(pose.orientation.roll(), 
                    pose.orientation.pitch(), 
                    pose.orientation.yaw());
    quat_new.normalize();
    tf2::fromMsg(marker->pose.orientation, quat_model);
    tf2::Quaternion quat_updated = quat_new * quat_model;
    quat_updated.normalize();
    marker->pose.orientation = tf2::toMsg(quat_updated);

    return std::move(marker);
}
