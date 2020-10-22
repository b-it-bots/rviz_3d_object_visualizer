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

#ifndef OBJECT_VIS_MANAGER
#define OBJECT_VIS_MANAGER

#include <map>
#include <memory>

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

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

    typedef std::map<int, std::shared_ptr<MarkerInfo>> MarkerInfoMap;
    typedef std::map<int, std::shared_ptr<MarkerInfo>>::iterator MarkerInfoMapItr;
    typedef std::map<int, std::shared_ptr<MarkerInfo>>::const_iterator MarkerInfoMapConstItr;
    typedef std::pair<int, std::shared_ptr<MarkerInfo>> MarkerInfoPair;

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
    std::unique_ptr<rviz::BoolProperty> addObject(const std::string& categoryName, 
        const std::string& name, int uniqueId);
    std::unique_ptr<rviz::BoolProperty> addPerson(const std::string& name, int uniqueId);
    std::unique_ptr<rviz::BoolProperty> addPlane(const std::string& name, int uniqueId);

    void markerArrayCb(const visualization_msgs::MarkerArray::ConstPtr& msg);

    BaseTypes getBaseType(const visualization_msgs::Marker& msg);
    std::string getObjectCategory(const visualization_msgs::Marker& msg);
    std::string getDisplayName(const visualization_msgs::Marker& msg);

    void addNewMarker(const visualization_msgs::Marker& msg);
    void updateMarker(const visualization_msgs::Marker& msg);
    void deleteMarker(const int marker_id);

    std::unique_ptr<Ogre::SceneNode> createSceneNode(const visualization_msgs::Marker& msg);
    std::unique_ptr<rviz::MarkerBase> createMarker(const visualization_msgs::Marker& msg,
                                   Ogre::SceneNode* scene_node);
    std::unique_ptr<rviz::BoolProperty> createProperty(const visualization_msgs::Marker& msg);

    ros::Subscriber marker_array_sub_;
    rviz::MarkerDisplay marker_display_;

    std::vector<std::shared_ptr<Ogre::SceneNode>> base_scene_nodes_;
    std::map<std::string, std::shared_ptr<Ogre::SceneNode>> obj_category_scene_nodes_;

    std::vector<std::shared_ptr<rviz::Property>> base_properties_;
    std::map<std::string, std::shared_ptr<rviz::BoolProperty>> obj_category_properties_;

    MarkerInfoMap marker_store_;
};

class MarkerInfo
{
public:
    MarkerInfo();
    MarkerInfo(int unique_id, std::shared_ptr<Ogre::SceneNode> scene_node, 
               std::shared_ptr<rviz::MarkerBase> marker, 
               std::shared_ptr<rviz::BoolProperty> property,
               bool visibility = true);
    virtual ~MarkerInfo(){}

    void setVisible(bool visibility);
    bool getVisibility() const { return visibility_; }
    void updateVisibility(const ObjectVisualizationManager::MarkerInfoMap& marker_store);

    void updateMarker(std::shared_ptr<rviz::MarkerBase> marker);

    int unique_id_;
    std::shared_ptr<Ogre::SceneNode> scene_node_;
    std::shared_ptr<rviz::MarkerBase> marker_;
    std::shared_ptr<rviz::BoolProperty> property_;

protected:
    bool visibility_;
};

} // end namespace RVizVisualization

#endif // OBJECT_VIS_MANAGER
