/**
    File: mdr_dataloader.h
    Purpose: Loader for b-it-bots MAS MongoDB data
    @author Ahmed Faisal Abdelrahman
    @author Sushant Vijay Chavan
    @version 1.0
*/

#ifndef MDR_DATALOADER_H
#define MDR_DATALOADER_H

#include <ros/ros.h>
#include "dataloader/abstract_dataloader.h"
#include "dataloader/modelloader/model_loader.h"
#include "dataloader/modelloader/model_data.h"

#include <mas_perception_msgs/Object.h>
#include <mas_perception_msgs/Person.h>
#include <mas_perception_msgs/Plane.h>

namespace RVizDataLoader 
{
    class MDRDataloader: public AbstractDataloader
    {
        public:
            MDRDataloader(ros::NodeHandle nh);
            virtual ~MDRDataloader();

            virtual void queryDatabase();
            virtual void runDataUpdateLoop();
            void printStoredObjectData();
            void publishObjectData();

            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Person const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Object const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Plane const& data); 

            template <typename T>
            void updateObjectData()
            {
                std::string object_id;
                bool object_in_database = false;
                std::vector<boost::shared_ptr<T>> queried_objects;

                message_proxy_.query<T>(queried_objects);
                for (int i = 0; i < queried_objects.size(); i++)
                {
                    object_id = queried_objects[i]->name;
                    for (auto it = object_data.begin(); it != object_data.end(); it++)
                        if (object_id == it->first) object_in_database = true;
                    if (object_in_database) 
                    {
                        object_in_database = false;
                        continue;
                    }
                    std::cout << *queried_objects[i] << std::endl;
                    /* ModelData model_data = ModelData(); */
                    /* model_data.pose = Utils::Pose<double>(queried_objects[i]->pose.pose.position, queried_objects[i]->pose.pose.orientation); */
                    object_data.insert(std::pair<std::string, ModelData>(object_id, ModelData()));
                }
            }
    
        private:
            int update_loop_rate;
            int item_id;
            ros::Publisher object_data_pub;
            std::map<std::string, ModelData> object_data;
            ModelLoader* model_loader;
    };

    // TODO: transfer to and get working in cpp:
    std::ostream& operator<< (std::ostream &out, mas_perception_msgs::Person const& data) 
    {
        out << "Type: Person" << std::endl;
        out << "ID: " << data.id << std::endl;
        out << "Pose: " << std::endl;
        out << "  - Position: " << std::endl;
        out << "    - x: " << data.pose.pose.position.x << std::endl;
        out << "    - y: " << data.pose.pose.position.y << std::endl;
        out << "    - z: " << data.pose.pose.position.z << std::endl;
        out << "Height: " << data.height << std::endl;
        out << "Width: " << data.width << std::endl;
        return out;
    }

    std::ostream& operator<< (std::ostream &out, mas_perception_msgs::Object const& data) 
    {
        out << "Type: Object" << std::endl;
        out << "Database ID: " << data.database_id << std::endl;
        out << "Name: " << data.name << std::endl;
        out << "Pose: " << std::endl;
        out << "  - Position: " << std::endl;
        out << "    - x: " << data.pose.pose.position.x << std::endl;
        out << "    - y: " << data.pose.pose.position.y << std::endl;
        out << "    - z: " << data.pose.pose.position.z << std::endl;
        out << "  - Dimensions: " << std::endl;
        out << "    - x: " << data.dimensions.vector.x << std::endl;
        out << "    - y: " << data.dimensions.vector.y << std::endl;
        out << "    - z: " << data.dimensions.vector.z << std::endl;
        return out;
    }

    std::ostream& operator<< (std::ostream &out, mas_perception_msgs::Plane const& data) 
    {
        out << "Type: Plane" << std::endl;
        out << "Plane Point: " << std::endl;
        out << "  - x: " << data.plane_point.x << std::endl;
        out << "  - y: " << data.plane_point.y << std::endl;
        out << "  - z: " << data.plane_point.z << std::endl;
        out << "Object List: " << std::endl;
        for (auto object: data.object_list.objects)
        {
            out << "  - " << object.name << std::endl;
        }
        return out;
    }
}

#endif
