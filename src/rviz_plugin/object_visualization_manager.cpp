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

#include <OgreSceneManager.h>
#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/default_plugin/markers/mesh_resource_marker.h>
#include <rviz/default_plugin/markers/triangle_list_marker.h>
#include <rviz/default_plugin/markers/text_view_facing_marker.h>
#include <rviz/properties/property_tree_widget.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/bool_property.h>
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

MarkerInfo::MarkerInfo(int unique_id,
                       std::shared_ptr<Ogre::SceneNode> scene_node,
                       std::shared_ptr<MarkerBase> marker, 
                       std::shared_ptr<BoolProperty> property,
                       bool visibility)
: unique_id_(unique_id)
, scene_node_(scene_node)
, marker_(marker)
, property_(property)
, visibility_(visibility)
{
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

void MarkerInfo::updateMarker(std::shared_ptr<rviz::MarkerBase> marker)
{
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
    base_scene_nodes_.resize(BaseTypes::COUNT);

    base_scene_nodes_[BaseTypes::ROOT].reset(
        vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode("ObjectVisualizationRootSceneNode"));

    base_scene_nodes_[BaseTypes::OBJECTS].reset(
        base_scene_nodes_[BaseTypes::ROOT]->createChildSceneNode("ObjectsSceneNode"));

    base_scene_nodes_[BaseTypes::PERSONS].reset(
        base_scene_nodes_[BaseTypes::ROOT]->createChildSceneNode("PersonsSceneNode"));

    base_scene_nodes_[BaseTypes::PLANES].reset(
        base_scene_nodes_[BaseTypes::ROOT]->createChildSceneNode("PlanesSceneNode"));
}

void ObjectVisualizationManager::setupBaseProperties()
{
    base_properties_.resize(BaseTypes::COUNT);
    base_properties_[BaseTypes::ROOT].reset(new rviz::Property(QString::fromStdString("ObjectVisualizationRootProperty")));

    // Create the base properties for different types
    base_properties_[BaseTypes::OBJECTS].reset(new rviz::BoolProperty(
        QString::fromStdString(std::string("Objects")), true,
        "Object 3D markers", base_properties_[BaseTypes::ROOT].get(), SLOT(updateMarkerVisibilities()), this));
    base_properties_[BaseTypes::PERSONS].reset(new rviz::BoolProperty(
        QString::fromStdString(std::string("Persons")), true,
        "Persons 3D markers", base_properties_[BaseTypes::ROOT].get(), SLOT(updateMarkerVisibilities()), this));
    base_properties_[BaseTypes::PLANES].reset(new rviz::BoolProperty(
        QString::fromStdString(std::string("Planes")), true,
        "Planes 3D markers", base_properties_[BaseTypes::ROOT].get(), SLOT(updateMarkerVisibilities()), this));

    rviz::PropertyTreeModel* model = new rviz::PropertyTreeModel(base_properties_[BaseTypes::ROOT].get(), this);
    rviz::PropertyTreeWidget* tree_widget = new rviz::PropertyTreeWidget();
    tree_widget->setModel(model);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(tree_widget);
    setLayout(layout);
}

void ObjectVisualizationManager::updateMarkerVisibilities()
{
    for (auto& marker_info: marker_store_)
    {
        marker_info.second->updateVisibility(marker_store_);
    }

    for (auto& item: obj_category_properties_)
    {
        std::string category = item.first;
        obj_category_scene_nodes_[category]->setVisible(item.second->getBool(), !item.second->getBool());
    }

    for (int i = BaseTypes::OBJECTS; i < BaseTypes::COUNT; i++)
    {
        base_scene_nodes_[i]->setVisible(std::static_pointer_cast<BoolProperty>(base_properties_[i])->getBool(),
                                        !std::static_pointer_cast<BoolProperty>(base_properties_[i])->getBool());
    }
}

void ObjectVisualizationManager::addNewObjectCategory(const std::string& categoryName)
{
    if (!obj_category_scene_nodes_[categoryName])
    {
        std::cout << "Creating Obj Category scene node" << std::endl;
        obj_category_scene_nodes_[categoryName].reset(base_scene_nodes_[BaseTypes::OBJECTS]->createChildSceneNode());
    }

    if (!obj_category_properties_[categoryName])
    {
        obj_category_properties_[categoryName].reset(new rviz::BoolProperty(
            QString::fromStdString(categoryName), true,
            categoryName.c_str(), base_properties_[BaseTypes::OBJECTS].get(), SLOT(updateMarkerVisibilities()), this));
    }
}

void ObjectVisualizationManager::removeEmptyObjectCategories()
{
    std::vector<std::string> delete_list;
    // Find the empty categories
    for (const auto& node: obj_category_scene_nodes_)
    {
        if (node.second->numChildren() <= 0)
        {
            delete_list.push_back(node.first);
        }
    }

    // Delete the empty categories
    for (const auto& category: delete_list)
    {
        obj_category_scene_nodes_.erase(category);
        obj_category_properties_.erase(category);
    }

}

std::unique_ptr<rviz::BoolProperty> ObjectVisualizationManager::addObject(const std::string& categoryName, const std::string& name, int uniqueId)
{
    if (!obj_category_properties_.count(categoryName) || !obj_category_scene_nodes_.count(categoryName))
    {
        addNewObjectCategory(categoryName);
    }

    std::unique_ptr<rviz::BoolProperty> prop(new rviz::BoolProperty(
        QString::fromStdString(name), true, name.c_str(), 
        obj_category_properties_[categoryName].get(), 
        SLOT(updateMarkerVisibilities()), this));
    return prop;
}

std::unique_ptr<rviz::BoolProperty> ObjectVisualizationManager::addPerson(const std::string& name, int uniqueId)
{
    std::unique_ptr<rviz::BoolProperty> prop(new rviz::BoolProperty(
        QString::fromStdString(name), true, name.c_str(), 
        base_properties_[BaseTypes::PERSONS].get(), 
        SLOT(updateMarkerVisibilities()), this));
    return prop;
}

std::unique_ptr<rviz::BoolProperty> ObjectVisualizationManager::addPlane(const std::string& name, int uniqueId)
{
    std::unique_ptr<rviz::BoolProperty> prop(new rviz::BoolProperty(
        QString::fromStdString(name), true,
        name.c_str(), base_properties_[BaseTypes::PLANES].get(), SLOT(updateMarkerVisibilities()), this));
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

    removeEmptyObjectCategories();
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

std::unique_ptr<rviz::BoolProperty> ObjectVisualizationManager::createProperty(const visualization_msgs::Marker& msg)
{
    std::unique_ptr<rviz::BoolProperty> prop(nullptr);
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

std::unique_ptr<Ogre::SceneNode> ObjectVisualizationManager::createSceneNode(const visualization_msgs::Marker& msg)
{
    std::unique_ptr<Ogre::SceneNode> scene_node = nullptr;
    switch (getBaseType(msg))
    {
    case BaseTypes::OBJECTS:
        scene_node.reset(obj_category_scene_nodes_[getObjectCategory(msg)]->createChildSceneNode());
        break;
    case BaseTypes::PERSONS:
        scene_node.reset(base_scene_nodes_[BaseTypes::PERSONS]->createChildSceneNode());
        break;
    case BaseTypes::PLANES:
        scene_node.reset(base_scene_nodes_[BaseTypes::PLANES]->createChildSceneNode());
        break;
    default:
        ROS_ERROR("The marker tag does not have the right type! Cannot create Ogre scene node!");
        break;
    }
    return scene_node;
}

std::unique_ptr<rviz::MarkerBase> ObjectVisualizationManager::createMarker(const visualization_msgs::Marker& msg, Ogre::SceneNode* scene_node)
{
    std::unique_ptr<rviz::MarkerBase> marker = nullptr;
    visualization_msgs::Marker marker_msg = msg;
    switch (marker_msg.type)
    {
    case visualization_msgs::Marker::MESH_RESOURCE:
        marker.reset(new MeshResourceMarker(&marker_display_, vis_manager_, scene_node));
        break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
        marker.reset(new TriangleListMarker(&marker_display_, vis_manager_, scene_node));
        break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
        marker.reset(new TextViewFacingMarker(&marker_display_, vis_manager_, scene_node));
        marker_msg.text = getDisplayName(marker_msg);
        break;
    default:
        ROS_ERROR("[ObjectVisualizationManager] Unsupported marker type!");
        break;
    }

    if (marker)
    {
        marker->setMessage(marker_msg);
    }

    return marker;
}

void ObjectVisualizationManager::addNewMarker(const visualization_msgs::Marker& msg)
{
    
    std::unique_ptr<BoolProperty> property = createProperty(msg);
    std::unique_ptr<Ogre::SceneNode> scene_node = createSceneNode(msg);
    std::unique_ptr<rviz::MarkerBase> marker = createMarker(msg, scene_node.get());

    marker_store_.insert(MarkerInfoPair(msg.id, std::shared_ptr<MarkerInfo>(
        new MarkerInfo(msg.id, std::move(scene_node), std::move(marker), std::move(property)))));
    ROS_INFO("Added new marker with id: %d", msg.id);
}

void ObjectVisualizationManager::updateMarker(const visualization_msgs::Marker& msg)
{
    std::shared_ptr<MarkerInfo> marker_info = marker_store_.at(msg.id);
    marker_info->updateMarker(createMarker(msg, marker_info->scene_node_.get()));
    updateMarkerVisibilities();
}

void ObjectVisualizationManager::deleteMarker(const int marker_id)
{
    MarkerInfoMapItr marker_itr = marker_store_.find(marker_id);
    if (marker_itr != marker_store_.end())
    {
        ROS_INFO("Deleting marker with id %d!", marker_id);
        marker_itr->second = nullptr;
        marker_store_.erase(marker_id);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RVizVisualization::ObjectVisualizationManager, rviz::Panel)
