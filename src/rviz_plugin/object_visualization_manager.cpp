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

/**
  File: object_visualization_manager.cpp
  Purpose: RViz plugin for viewing and filtering the 3D meshes
  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/default_plugin/markers/mesh_resource_marker.h>
#include <rviz/default_plugin/markers/triangle_list_marker.h>
#include <rviz/default_plugin/markers/text_view_facing_marker.h>

#include <rviz/properties/property_tree_widget.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/bool_property.h>

#include <QVBoxLayout>

#include <visualization_msgs/Marker.h>

#include "rviz_plugin/object_visualization_manager.h"

using namespace rviz;
using namespace RVizVisualization;

MarkerInfo::MarkerInfo()
: unique_id_(-1)
, scene_node_(nullptr)
, marker_(nullptr)
, property_(nullptr)
, visibility_(true)
{
}

MarkerInfo::MarkerInfo(int unique_id, Ogre::SceneNode* scene_node,
                       MarkerBase* marker, BoolProperty* property,
                       bool visibility)
: unique_id_(unique_id)
, scene_node_(scene_node)
, marker_(marker)
, property_(property)
, visibility_(visibility)
{
}

MarkerInfo::~MarkerInfo()
{
    if (scene_node_)
    {
        delete scene_node_;
        scene_node_ = nullptr;
    }

    if (marker_)
    {
        delete marker_;
        marker_ = nullptr;
    }

    if (property_)
    {
        delete property_;
        property_ = nullptr;
    }
}

void MarkerInfo::setVisible(bool visibility)
{
    visibility_ = visibility;
    scene_node_->setVisible(visibility_);
}

void MarkerInfo::updateVisibility(const ObjectVisualizationManager::MarkerInfoMap& marker_store)
{
    if (property_)
    {
        setVisible(property_->getBool());
    }
    else
    {
        ObjectVisualizationManager::MarkerInfoMapConstItr itr = marker_store.find(unique_id_-1);
        if (itr != marker_store.end())
        {
            setVisible(itr->second->property_->getBool());
        }
    }
    
}

void MarkerInfo::updateMarker(MarkerBase* marker)
{
    // Delete the existing marker object if it exists
    if (marker_)
    {
        delete marker_;
        marker_ = nullptr;
    }
    marker_ = marker;
    scene_node_->setVisible(visibility_);
}

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
    setupBaseProperties();
    base_scene_nodes.resize(BaseTypes::COUNT);
    base_scene_nodes[BaseTypes::ROOT] = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode("ObjectVisualizationRootSceneNode");
    base_scene_nodes[BaseTypes::OBJECTS] = base_scene_nodes[BaseTypes::ROOT]->createChildSceneNode("ObjectsSceneNode");
    base_scene_nodes[BaseTypes::PERSONS] = base_scene_nodes[BaseTypes::ROOT]->createChildSceneNode("PersonsSceneNode");
    base_scene_nodes[BaseTypes::PLANES] = base_scene_nodes[BaseTypes::ROOT]->createChildSceneNode("PlanesSceneNode");
}

void ObjectVisualizationManager::setupBaseProperties()
{
    base_properties.resize(BaseTypes::COUNT);
    base_properties[BaseTypes::ROOT] = new rviz::Property(QString::fromStdString("ObjectVisualizationRootProperty"));

    // Create the base properties for different types
    base_properties[BaseTypes::OBJECTS] = new rviz::BoolProperty(
        QString::fromStdString(std::string("Objects")), true,
        "Object 3D markers", base_properties[BaseTypes::ROOT], SLOT(updateMarkerVisibilities()), this);
    base_properties[BaseTypes::PERSONS] = new rviz::BoolProperty(
        QString::fromStdString(std::string("Persons")), true,
        "Persons 3D markers", base_properties[BaseTypes::ROOT], SLOT(updateMarkerVisibilities()), this);
    base_properties[BaseTypes::PLANES] = new rviz::BoolProperty(
        QString::fromStdString(std::string("Planes")), true,
        "Planes 3D markers", base_properties[BaseTypes::ROOT], SLOT(updateMarkerVisibilities()), this);

    rviz::PropertyTreeModel* model = new rviz::PropertyTreeModel(base_properties[BaseTypes::ROOT], this);
    tree_widget_ = new rviz::PropertyTreeWidget();
    tree_widget_->setModel(model);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(tree_widget_);
    setLayout(layout);
}

void ObjectVisualizationManager::updateMarkerVisibilities()
{
    for (auto& markerInfo: marker_store_)
    {
        markerInfo.second->updateVisibility(marker_store_);
    }

    for (auto& item: obj_category_properties)
    {
        std::string category = item.first;
        obj_category_scene_nodes[category]->setVisible(item.second->getBool(), !item.second->getBool());
    }

    for (int i = BaseTypes::OBJECTS; i < BaseTypes::COUNT; i++)
    {
        base_scene_nodes[i]->setVisible(static_cast<BoolProperty*>(base_properties[i])->getBool(),
                                        !static_cast<BoolProperty*>(base_properties[i])->getBool());
    }
}

void ObjectVisualizationManager::addNewObjectCategory(const std::string& categoryName)
{
    if (!obj_category_scene_nodes[categoryName])
    {
        std::cout << "Creating Obj Category scene node" << std::endl;
        obj_category_scene_nodes[categoryName] = base_scene_nodes[BaseTypes::OBJECTS]->createChildSceneNode();
    }

    if (!obj_category_properties[categoryName])
    {
        obj_category_properties[categoryName] = new rviz::BoolProperty(
            QString::fromStdString(categoryName), true,
            categoryName.c_str(), base_properties[BaseTypes::OBJECTS], SLOT(updateMarkerVisibilities()), this);
    }
}

rviz::BoolProperty* ObjectVisualizationManager::addObject(const std::string& categoryName, const std::string& name, int uniqueId)
{
    if (!obj_category_properties.count(categoryName) || !obj_category_scene_nodes.count(categoryName))
    {
        addNewObjectCategory(categoryName);
    }

    rviz::BoolProperty* prop = new rviz::BoolProperty(
        QString::fromStdString(name), true,
        name.c_str(), obj_category_properties[categoryName], SLOT(updateMarkerVisibilities()), this);
    return prop;
}

rviz::BoolProperty* ObjectVisualizationManager::addPerson(const std::string& name, int uniqueId)
{
    rviz::BoolProperty* prop = new rviz::BoolProperty(
        QString::fromStdString(name), true,
        name.c_str(), base_properties[BaseTypes::PERSONS], SLOT(updateMarkerVisibilities()), this);
    return prop;
}

rviz::BoolProperty* ObjectVisualizationManager::addPlane(const std::string& name, int uniqueId)
{
    rviz::BoolProperty* prop = new rviz::BoolProperty(
        QString::fromStdString(name), true,
        name.c_str(), base_properties[BaseTypes::PLANES], SLOT(updateMarkerVisibilities()), this);
    return prop;
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
    //ROS_INFO("Marker Array received!");
    for(const auto& marker : msg->markers)
    {
        if (marker.action == visualization_msgs::Marker::ADD)
        {
            MarkerInfoMapItr itr  = marker_store_.find(marker.id);
            if (itr == marker_store_.end())
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

ObjectVisualizationManager::BaseTypes ObjectVisualizationManager::getBaseType(const visualization_msgs::Marker& msg)
{
    std::string tag = msg.text;
    std::string type = tag.substr(0, tag.find('/'));
    if (type == "OBJECT")
        return BaseTypes::OBJECTS;
    else if (type == "PERSON")
        return BaseTypes::PERSONS;
    else if (type == "PLANE")
        return BaseTypes::PLANES;
    else
        return BaseTypes::COUNT;
}

std::string ObjectVisualizationManager::getObjectCategory(const visualization_msgs::Marker& msg)
{
    std::string tag = msg.text;
    std::string sub_tag = tag.substr(tag.find('/')+1, tag.size());
    std::string category = sub_tag.substr(0, sub_tag.find('/'));
    return category;
}

std::string ObjectVisualizationManager::getDisplayName(const visualization_msgs::Marker& msg)
{
    std::string tag = msg.text;
    std::string name = tag.substr(tag.rfind('/')+1, tag.size());
    return name;
}

BoolProperty* ObjectVisualizationManager::createProperty(const visualization_msgs::Marker& msg)
{
    BoolProperty* prop = nullptr;
    if (msg.type != visualization_msgs::Marker::TEXT_VIEW_FACING)
    {
        switch (getBaseType(msg))
        {
        case BaseTypes::OBJECTS:
            prop = addObject(getObjectCategory(msg), getDisplayName(msg), msg.id);
            break;
        case BaseTypes::PERSONS:
            prop = addPerson(getDisplayName(msg), msg.id);
            break;
        case BaseTypes::PLANES:
            prop = addPlane(getDisplayName(msg), msg.id);
            break;
        default:
            ROS_ERROR("The marker tag does not have the right type! Cannot create RViz property!");
            break;
        }
    }
    return prop;
}

Ogre::SceneNode* ObjectVisualizationManager::createSceneNode(const visualization_msgs::Marker& msg)
{
    Ogre::SceneNode* scene_node = nullptr;
    switch (getBaseType(msg))
    {
    case BaseTypes::OBJECTS:
        scene_node = obj_category_scene_nodes[getObjectCategory(msg)]->createChildSceneNode();
        break;
    case BaseTypes::PERSONS:
        scene_node = base_scene_nodes[BaseTypes::PERSONS]->createChildSceneNode();
        break;
    case BaseTypes::PLANES:
        scene_node = base_scene_nodes[BaseTypes::PLANES]->createChildSceneNode();
        break;
    default:
        ROS_ERROR("The marker tag does not have the right type! Cannot create Ogre scene node!");
        break;
    }
    return scene_node;
}

rviz::MarkerBase* ObjectVisualizationManager::createMarker(const visualization_msgs::Marker& msg, Ogre::SceneNode* scene_node)
{
    MarkerBase* marker = nullptr;
    visualization_msgs::Marker markerMsg = msg;
    switch (markerMsg.type)
    {
    case visualization_msgs::Marker::MESH_RESOURCE:
        marker = new MeshResourceMarker(&marker_display_, vis_manager_, scene_node);
        break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
        marker = new TriangleListMarker(&marker_display_, vis_manager_, scene_node);
        break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        marker = new TextViewFacingMarker(&marker_display_, vis_manager_, scene_node);
        markerMsg.text = getDisplayName(markerMsg);
        break;
    default:
        ROS_ERROR("[ObjectVisualizationManager] Unsupported marker type!");
        break;
    }

    if (marker)
    {
        marker->setMessage(markerMsg);
    }

    return marker;
}

void ObjectVisualizationManager::addNewMarker(const visualization_msgs::Marker& msg)
{
    BoolProperty* property = createProperty(msg);
    Ogre::SceneNode* scene_node = createSceneNode(msg);
    rviz::MarkerBase* marker = createMarker(msg, scene_node);

    marker_store_.insert(MarkerInfoPair(msg.id, new MarkerInfo(msg.id, scene_node, marker, property)));
    ROS_INFO("Added new marker with id: %d", msg.id);
}

void ObjectVisualizationManager::updateMarker(const visualization_msgs::Marker& msg)
{
    MarkerInfo* markerInfo = marker_store_.at(msg.id);
    markerInfo->updateMarker(createMarker(msg, markerInfo->scene_node_));
    updateMarkerVisibilities();
}

void ObjectVisualizationManager::deleteMarker(const int marker_id)
{
    MarkerInfoMapItr markerItr = marker_store_.find(marker_id);
    if (markerItr != marker_store_.end())
    {
        ROS_INFO("Deleting marker with id %d!", marker_id);
        delete markerItr->second;
        markerItr->second = nullptr;
        marker_store_.erase(marker_id);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RVizVisualization::ObjectVisualizationManager, rviz::Panel)
