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

    std::unique_ptr<MeshData> mesh_data = yaml_loader_.getMeshConfig(mesh_type);

    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->pose.position.x = mesh_data->pose_.position.x();
    marker->pose.position.y = mesh_data->pose_.position.y();
    marker->pose.position.z = mesh_data->pose_.position.z();
    marker->pose.orientation = tf2::toMsg(Utils::toTf2Quaternion(mesh_data->pose_.orientation));

    marker->scale.x = mesh_data->scale_.x();
    marker->scale.y = mesh_data->scale_.y();
    marker->scale.z = mesh_data->scale_.z();

    if (mesh_data->use_color_from_mesh_)
    {
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 0.0;
        marker->color.a = 0.0;
        marker->mesh_use_embedded_materials = true;
    }
    else
    {
        marker->color.r = mesh_data->color_.r() / 255.0;
        marker->color.g = mesh_data->color_.g() / 255.0;
        marker->color.b = mesh_data->color_.b() / 255.0;
        marker->color.a = 1.0;
        marker->mesh_use_embedded_materials = false;
    }

    marker->mesh_resource = mesh_data->mesh_resource_;

    return std::move(marker);
}

auto ModelLoader::getTextLabelMarker(const std::string& name,
                        const Utils::Vec3<double>& pos) -> std::unique_ptr<visualization_msgs::Marker>
{
    std::unique_ptr<visualization_msgs::Marker> marker = std::unique_ptr<visualization_msgs::Marker>(new visualization_msgs::Marker);

    marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker->text = name; //name.substr(name.rfind("/") + 1, name.size());
    marker->pose.position.x = pos.x();
    marker->pose.position.y = pos.y();
    marker->pose.position.z = pos.z();

    marker->scale.z = 0.1;

    marker->color.r = 1.0;
    marker->color.g = 0.0;
    marker->color.b = 0.0;
    marker->color.a = 1.0;

    return std::move(marker);
}

auto ModelLoader::getMeshMarker(int id, Mesh::Types type,
                                const std::string& name,
                                const std::string& frame_id,
                                const std::string& ns,
                                const Utils::Pose<double>& pose)
                                -> MarkerResultPair
{
    if (id % 2 != 0)
    {
        ROS_ERROR("Marker ID must be an even number! Odd numver ID's are \
                   reserved for the text labels. Request to create a marker \
                   with ID %d and name %s has been ignored!", id, name.c_str());
        return MarkerResultPair(nullptr, nullptr);
    }

    // Create the mesh marker
    std::unique_ptr<visualization_msgs::Marker> marker = loadModel(type);
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = ns;
    marker->id = id;
    marker->text = name;

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

    //Create the text label marker
    std::unique_ptr<MeshData> mesh_data = yaml_loader_.getMeshConfig(type);
    Utils::Vec3<double> pos = mesh_data->text_offset_;
    pos.update(pos.x() + pose.position.x(), pos.y() + pose.position.y(), pos.z() + pose.position.z());
    std::unique_ptr<visualization_msgs::Marker> textMarker = getTextLabelMarker(name, pos);
    textMarker->header.frame_id = frame_id;
    textMarker->header.stamp = ros::Time();
    textMarker->ns = ns;
    textMarker->id = id+1;

    return MarkerResultPair(std::move(marker), std::move(textMarker));
}

auto ModelLoader::getPlaneMarker(int id,
                                 const std::string& name,
                                 const std::string& frame_id, 
                                 const std::string& ns,
                                 const Utils::Vec3<double>& center,
                                 const Utils::Vec3Array<double>& convex_hull,
                                 const Utils::Vec4<double>& color,
                                 const Utils::Vec3<double>& scale)
                                 -> MarkerResultPair
{
    if (id % 2 != 0)
    {
        ROS_ERROR("Marker ID must be an even number! Odd numver ID's are \
                   reserved for the text labels. Request to create a marker \
                   with ID %d and name %s has been ignored!", id, name.c_str());
        return MarkerResultPair(nullptr, nullptr);
    }

    std::unique_ptr<visualization_msgs::Marker> marker = std::unique_ptr<visualization_msgs::Marker>(new visualization_msgs::Marker);

    marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = ns;
    marker->id = id;
    marker->text = name;

    // Set the vertices for the triangles forming the plane
    marker->points = generateTraingleVertices(center, convex_hull);

    // Set the color for the plane
    marker->color.r = color.r();
    marker->color.g = color.g();
    marker->color.b = color.b();
    marker->color.a = color.a();
    marker->mesh_use_embedded_materials = false;

    // Set the scaling for the plane
    marker->scale.x = scale.x();
    marker->scale.y = scale.y();
    marker->scale.z = scale.z();

    //Create the text label marker
    Utils::Vec3<double> pos(0, 0, 0.1);
    pos.update(pos.x() + center.x(), pos.y() + center.y(), pos.z() + center.z());
    std::unique_ptr<visualization_msgs::Marker> textMarker = getTextLabelMarker(name, pos);
    textMarker->header.frame_id = frame_id;
    textMarker->header.stamp = ros::Time();
    textMarker->ns = ns;
    textMarker->id = id+1;

    return MarkerResultPair(std::move(marker), std::move(textMarker));
}

auto ModelLoader::generateTraingleVertices(const Utils::Vec3<double>& center,
                                       const Utils::Vec3Array<double>& convex_hull)
                                       -> std::vector<geometry_msgs::Point>
{
    std::vector<geometry_msgs::Point> points;

    if (convex_hull.size() > 3)
    {
        // Construct a polygon
        points.reserve(convex_hull.size() * 3);

        for (int i = 0; i < convex_hull.size() - 1; i++)
        {
            // Points for each triangle need to be specified in counter-clockwise direction
            points.push_back(asRVizPoint(center));
            points.push_back(asRVizPoint(convex_hull[i + 1]));
            points.push_back(asRVizPoint(convex_hull[i]));
        }

        // Complete the last triangle
        points.push_back(asRVizPoint(center));
        points.push_back(asRVizPoint(convex_hull[0]));
        points.push_back(asRVizPoint(convex_hull[convex_hull.size() - 1]));
    }
    else if (convex_hull.size() == 3)
    {
        // Construct a Triangle
        points.push_back(asRVizPoint(convex_hull[0]));
        points.push_back(asRVizPoint(convex_hull[2]));
        points.push_back(asRVizPoint(convex_hull[1]));
    }
    else
    {
        ROS_ERROR("Convex hull must have atleast three points to render as a plane!!");
    }

    return points;
}

geometry_msgs::Point ModelLoader::asRVizPoint(const Utils::Vec3<double>& point)
{
    geometry_msgs::Point p;

    p.x = point.x();
    p.y = point.y();
    p.z = point.z();

    return p;
}
