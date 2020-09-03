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
#include <typeinfo>

#include <rviz/visualization_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/default_plugin/markers/mesh_resource_marker.h>
#include <geometry_msgs/Pose.h>
#include "rviz_plugin/object_visualization_manager.h"

using namespace rviz;
using namespace RVizVisualization;

ObjectVisualizationManager::ObjectVisualizationManager(QWidget* parent)
: rviz::Panel(parent)
{
    ros::NodeHandle nh;
    marker_array_sub_ = nh.subscribe<visualization_msgs::MarkerArray> ("/ObjectVisualizationManager/MarkerArray", 10, &ObjectVisualizationManager::markerArrayCb, this);
}

ObjectVisualizationManager::~ObjectVisualizationManager()
{
}

void ObjectVisualizationManager::onInitialize()
{
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
    for(const auto& marker : msg->markers)
    {
        addMarker(marker);
    }
}

void ObjectVisualizationManager::addMarker(const visualization_msgs::Marker& msg)
{
    MeshResourceMarker* mesh = new MeshResourceMarker(&marker_display_, vis_manager_, vis_manager_->getSceneManager()->getRootSceneNode());
    mesh->setMessage(msg);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RVizVisualization::ObjectVisualizationManager, rviz::Panel)
