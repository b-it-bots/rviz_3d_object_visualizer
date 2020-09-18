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
  File: object_visualization_manager.h
  Purpose: RViz plugin for viewing and filtering 3D object meshes

  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 15/08/20
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

    std::vector<Ogre::SceneNode*> base_scene_nodes;
    std::map<std::string, Ogre::SceneNode*> obj_category_scene_nodes;

    std::vector<rviz::Property*> base_properties;
    std::map<std::string, rviz::BoolProperty*> obj_category_properties;
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
