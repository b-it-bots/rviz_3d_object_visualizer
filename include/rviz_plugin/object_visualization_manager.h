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
  File: object_visualization_manager.h
  Purpose: RViz plugin for viewing and filtering the 3D meshes
  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

#ifndef OBJECT_VIS_MANAGER
#define OBJECT_VIS_MANAGER

#include <map>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/default_plugin/marker_display.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/default_plugin/markers/marker_base.h>

#include <OgreSceneNode.h>

namespace rviz
{
    class Property;
    class BoolProperty;
    class PropertyTreeWidget;
}

namespace RVizVisualization
{

class MarkerInfo;

class ObjectVisualizationManager : public rviz::Panel
{
Q_OBJECT
public:
    ObjectVisualizationManager(QWidget* parent = 0);
    virtual ~ObjectVisualizationManager();

    virtual void onInitialize();

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

    typedef std::map<int, MarkerInfo*> MarkerInfoMap;
    typedef std::map<int, MarkerInfo*>::iterator MarkerInfoMapItr;
    typedef std::map<int, MarkerInfo*>::const_iterator MarkerInfoMapConstItr;
    typedef std::pair<int, MarkerInfo*> MarkerInfoPair;

private Q_SLOTS:
  void updateMarkerVisibilities();

protected:
    enum BaseTypes
    {
        ROOT = 0,
        OBJECTS,
        PERSONS,
        PLANES,
        COUNT
    };

    void setupBaseProperties();
    void addNewObjectCategory(const std::string& categoryName);
    rviz::BoolProperty* addObject(const std::string& categoryName, const std::string& name, int uniqueId);
    rviz::BoolProperty* addPerson(const std::string& name, int uniqueId);
    rviz::BoolProperty* addPlane(const std::string& name, int uniqueId);

    void markerArrayCb(const visualization_msgs::MarkerArray::ConstPtr& msg);

    BaseTypes getBaseType(const visualization_msgs::Marker& msg);
    std::string getObjectCategory(const visualization_msgs::Marker& msg);
    std::string getDisplayName(const visualization_msgs::Marker& msg);

    void addNewMarker(const visualization_msgs::Marker& msg);
    void updateMarker(const visualization_msgs::Marker& msg);
    void deleteMarker(const int marker_id);

    Ogre::SceneNode* createSceneNode(const visualization_msgs::Marker& msg);
    rviz::MarkerBase* createMarker(const visualization_msgs::Marker& msg,
                                   Ogre::SceneNode* scene_node);
    rviz::BoolProperty* createProperty(const visualization_msgs::Marker& msg);

    ros::Subscriber marker_array_sub_;
    rviz::MarkerDisplay marker_display_;

    std::vector<Ogre::SceneNode*> base_scene_nodes_;
    std::map<std::string, Ogre::SceneNode*> obj_category_scene_nodes_;

    std::vector<rviz::Property*> base_properties_;
    std::map<std::string, rviz::BoolProperty*> obj_category_properties_;
    rviz::PropertyTreeWidget* tree_widget_;

    MarkerInfoMap marker_store_;
};

class MarkerInfo
{
public:
    MarkerInfo();
    MarkerInfo(int unique_id, Ogre::SceneNode* scene_node, 
               rviz::MarkerBase* marker, rviz::BoolProperty* property,
               bool visibility = true);
    virtual ~MarkerInfo();

    void setVisible(bool visibility);
    bool getVisibility() const { return visibility_; }
    void updateVisibility(const ObjectVisualizationManager::MarkerInfoMap& marker_store);

    void updateMarker(rviz::MarkerBase* marker);

    int unique_id_;
    Ogre::SceneNode* scene_node_;
    rviz::MarkerBase* marker_;
    rviz::BoolProperty* property_;

protected:
    bool visibility_;
};

} // end namespace RVizVisualization

#endif // OBJECT_VIS_MANAGER
