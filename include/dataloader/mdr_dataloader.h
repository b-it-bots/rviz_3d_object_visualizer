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
            virtual void publishObjectData();
            void printStoredObjectData();

            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Person const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Object const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Plane const& data); 

            // Standard Solution:
            template <typename T>
            void updateObjectData()
            {
                bool object_in_queried_list;
                std::vector<boost::shared_ptr<T>> queried_objects;

                message_proxy_.query<T>(queried_objects);

                for (int i = 0; i < queried_objects.size(); i++)
                {
                    std::string object_name = queried_objects[i]->name;
                    /* object_name = typeid(T).name(); */
                    ModelData model_data = ModelData();
                    auto position = queried_objects[i]->pose.pose.position;
                    auto orientation = queried_objects[i]->pose.pose.orientation;
                    Utils::Vec3<double> rpy_vector = Utils::toRPY(Utils::Vec4<double>(orientation.x, orientation.y, orientation.z, orientation.w));
                    model_data.pose_ = Utils::Pose<double>(position.x, position.y, position.z, rpy_vector.x(), rpy_vector.y(), rpy_vector.z());

                    if (object_data_.find(object_name) == object_data_.end())
                    {
                        // object not found; insert it in map:
                        model_data.unique_id_ = item_id_++;
                        object_data_.insert(std::pair<std::string, ModelData>(object_name, model_data));
                        std::cout << *queried_objects[i] << std::endl;
                    }
                    else
                    {
                        // object found in map; update its data:
                        object_data_[object_name] = model_data;
                        std::cout << "Old object data updated in map" << std::endl;
                    }
                }

                for (auto &object_in_map : object_data_)
                {
                    object_in_queried_list = false;
                    for (auto &object_in_query : queried_objects)
                    {
                        if (object_in_map.first == object_in_query->name)
                        {
                            object_in_queried_list = true;
                            break;
                        }
                    }

                    if (!object_in_queried_list)
                    {
                        // object not found in queried list; add to delete list, and erase from map
                        marker_delete_list_.push_back(object_in_map.second.unique_id_);
                        item_delete_list_.push_back(object_in_map.first);
                        std::cout << "Object in map not found in queried_list. Removing..." << std::endl;
                    }
                }
            }

            //TODO: Implement set solution, and compare:

        private:
            int update_loop_rate_;
            int item_id_;
            std::vector<int> marker_delete_list_;
            std::vector<std::string> item_delete_list_;
            std::map<std::string, ModelData> object_data_;
            ModelLoader* model_loader_;
    };

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
