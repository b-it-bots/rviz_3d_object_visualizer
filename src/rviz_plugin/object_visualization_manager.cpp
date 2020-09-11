/*
 * Copyright © 2020 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, this
 *       list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this
 *       list of conditions and the following disclaimer in the documentation and/or
 *       other materials provided with the distribution.
 *     * Neither the name of “Hochschule Bonn-Rhein-Sieg” nor the names of its contributors
 *       may be used to endorse or promote products derived from this software without specific
 *       prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
  File: object_visualization_manager.cpp
  Purpose: RViz plugin for viewing and filtering 3D object meshes

  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 15/08/20
*/

#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/default_plugin/markers/mesh_resource_marker.h>
#include <rviz/default_plugin/markers/triangle_list_marker.h>
#include <rviz/default_plugin/markers/text_view_facing_marker.h>
#include <visualization_msgs/Marker.h>

#include "rviz_plugin/object_visualization_manager.h"

using namespace rviz;
using namespace RVizVisualization;

ObjectVisualizationManager::ObjectVisualizationManager(QWidget* parent)
: rviz::Panel(parent)
{
    ros::NodeHandle nh;
    marker_array_sub_ = nh.subscribe<visualization_msgs::MarkerArray> ("/rviz_3d_object_visualizer/markers", 10, &ObjectVisualizationManager::markerArrayCb, this);
}

ObjectVisualizationManager::~ObjectVisualizationManager()
{
}

void ObjectVisualizationManager::onInitialize()
{
    root_scene_node_ = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
}

void ObjectVisualizationManager::save( rviz::Config config ) const
{
    rviz::Panel::save(config);
}

void ObjectVisualizationManager::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
}

void ObjectVisualizationManager::markerArrayCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    ROS_INFO("Marker Array received!");
    for(const auto& marker : msg->markers)
    {
        if (marker.action == visualization_msgs::Marker::ADD)
        {
            MarkerMapItr itr  = markers_map_.find(marker.id);
            if (itr == markers_map_.end())
            {
                addNewMarker(marker);
            }
            else
            {
                updateMarker(marker);
            }
        }
        else if (marker.action == visualization_msgs::Marker::DELETE)
        {
            deleteMarker(marker.id);
        }
    }
}

rviz::MarkerBase* ObjectVisualizationManager::createMarker(const visualization_msgs::Marker& msg, Ogre::SceneNode* scene_node)
{
    MarkerBase* marker = nullptr;
    switch (msg.type)
    {
    case visualization_msgs::Marker::MESH_RESOURCE:
        marker = new MeshResourceMarker(&marker_display_, vis_manager_, scene_node);
        break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
        marker = new TriangleListMarker(&marker_display_, vis_manager_, scene_node);
        break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        marker = new TextViewFacingMarker(&marker_display_, vis_manager_, scene_node);
        break;
    default:
        ROS_ERROR("[ObjectVisualizationManager] Unsupported marker type!");
        break;
    }

    if (marker)
    {
        marker->setMessage(msg);
    }

    return marker;
}

void ObjectVisualizationManager::addNewMarker(const visualization_msgs::Marker& msg)
{
    Ogre::SceneNode* scene_node = root_scene_node_->createChildSceneNode();
    rviz::MarkerBase* marker = createMarker(msg, scene_node);
    scene_nodes_map_.insert(ScenePair(msg.id, scene_node));
    markers_map_.insert(MarkerPair(msg.id, marker));
    ROS_INFO("Added new marker with id: %d", msg.id);
}

void ObjectVisualizationManager::updateMarker(const visualization_msgs::Marker& msg)
{
    Ogre::SceneNode* scene_node = scene_nodes_map_.at(msg.id);
    rviz::MarkerBase* marker = createMarker(msg, scene_node);
    MarkerMapItr itr  = markers_map_.find(msg.id);
    if (itr != markers_map_.end())
    {
        delete itr->second;
        itr->second = marker;
    }
    ROS_INFO("Updated existing marker with id: %d", msg.id);
}

void ObjectVisualizationManager::deleteMarker(const int marker_id)
{
    MarkerMapItr markerItr  = markers_map_.find(marker_id);
    if (markerItr != markers_map_.end())
    {
        delete markerItr->second;
        markers_map_.erase(marker_id);
    }

    SceneMapItr sceneItr  = scene_nodes_map_.find(marker_id);
    if (sceneItr != scene_nodes_map_.end())
    {
        delete sceneItr->second;
        scene_nodes_map_.erase(marker_id);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RVizVisualization::ObjectVisualizationManager, rviz::Panel)
