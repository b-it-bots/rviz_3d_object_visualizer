/**
     File: mdr_dataloader.cpp
     Purpose: ...
     @author Ahmed Faisal Abdelrahman
     @author Sushant Vijay Chavan
     @version 1.0
 */

#include <geometry_msgs/Pose.h>
#include <mas_perception_msgs/Object.h>
#include <mas_perception_msgs/Person.h>
#include <mas_perception_msgs/Plane.h>

#include "dataloader/mdr_dataloader.h"

using namespace RVizDataLoader;

MDRDataloader::MDRDataloader(ros::NodeHandle nh) : MongoDBDataloader(nh)
{
}

MDRDataloader::~MDRDataloader()
{
}

void MDRDataloader::queryDatabase()
{
    // Adding new entries:
    geometry_msgs::Pose pose_msg;
    mas_perception_msgs::Object object_msg;
    mas_perception_msgs::Person person_msg;
    mas_perception_msgs::Plane plane_msg;

    std::string id = message_proxy_.insertNamed("test_pose_data", pose_msg);
    message_proxy_.insertNamed("test_object_data", object_msg);
    message_proxy_.insertNamed("test_person_data", person_msg);
    message_proxy_.insertNamed("test_plane_data", plane_msg);

    // Updating an entry:
    pose_msg.position.x = 999;
    message_proxy_.updateID(id, pose_msg);
    assert(message_proxy_.queryID<geometry_msgs::Pose>(id).first->position.x == 999);

    // Reading existing entries:
    std::vector< boost::shared_ptr<geometry_msgs::Pose>> queried_poses;
    std::vector< boost::shared_ptr<mas_perception_msgs::Person>> queried_people;
    message_proxy_.queryNamed<geometry_msgs::Pose>("test_pose_data", queried_poses, false);
    std::cout << "Found " << queried_poses.size() << " poses in database" << std::endl;
    std::cout << queried_poses[0]->position.x << std::endl;
    message_proxy_.queryNamed<mas_perception_msgs::Person>("test_person_data", queried_people, false);
    std::cout << "Found " << queried_people.size() << " people in database" << std::endl;
}

void MDRDataloader::runDataUpdateLoop()
{
}
