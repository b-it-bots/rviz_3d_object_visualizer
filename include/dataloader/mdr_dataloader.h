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

#ifndef MDR_DATALOADER_H
#define MDR_DATALOADER_H

#include <ros/ros.h>

#include <mas_perception_msgs/Object.h>
#include <mas_perception_msgs/Person.h>
#include <mas_perception_msgs/Plane.h>

#include "dataloader/abstract_dataloader.h"
#include "dataloader/modelloader/model_loader.h"
#include "dataloader/modelloader/model_data.h"

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
            void fillObjectCategoryMeshMap();
            Mesh::Types getObjectMeshType(std::string object_category);

            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Person const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Object const& data); 
            friend std::ostream& operator<<(std::ostream &out, mas_perception_msgs::Plane const& data); 

            std::unique_ptr<MeshData> fillPoseDetails(mas_perception_msgs::Object object)
            {
                std::unique_ptr<MeshData> mesh_data(new MeshData());
                auto position = object.pose.pose.position;
                auto orientation = object.pose.pose.orientation;
                Utils::Vec3<double> rpy_vector = Utils::toRPY(Utils::Vec4<double>(orientation.x, orientation.y, orientation.z, orientation.w));
                mesh_data->pose_ = Utils::Pose<double>(position.x, position.y, position.z, rpy_vector.x(), rpy_vector.y(), rpy_vector.z());
                mesh_data->type_ = getObjectMeshType(object.category);

                mesh_data->name_ = object.name;

                return mesh_data;
            }

            std::unique_ptr<MeshData> fillPoseDetails(mas_perception_msgs::Person person)
            {
                std::unique_ptr<MeshData> mesh_data(new MeshData());
                auto position = person.pose.pose.position;
                auto orientation = person.pose.pose.orientation;
                Utils::Vec3<double> rpy_vector = Utils::toRPY(Utils::Vec4<double>(orientation.x, orientation.y, orientation.z, orientation.w));
                mesh_data->pose_ = Utils::Pose<double>(position.x, position.y, position.z, rpy_vector.x(), rpy_vector.y(), rpy_vector.z());
                mesh_data->type_ = Mesh::Types::PERSON;
                mesh_data->name_ = person.name;

                return mesh_data;
            }

            std::unique_ptr<PlaneData> fillPoseDetails(mas_perception_msgs::Plane plane)
            {
                std::unique_ptr<PlaneData> plane_data(new PlaneData());
                plane_data->center_ = Utils::Vec3<double>(plane.plane_point.x, plane.plane_point.y, plane.plane_point.z);
                for (auto &point: plane.convex_hull)
                {
                    plane_data->convex_hull_.push_back(Utils::Vec3<double>(point.x, point.y, point.z));
                }
                plane_data->name_ = plane.name;

                return plane_data;
            }

            template <typename T>
            void updateObjectData()
            {
                bool object_in_queried_list;
                std::vector<boost::unique_ptr<T>> queried_objects;

                message_proxy_.query<T>(queried_objects);

                for (int i = 0; i < queried_objects.size(); i++)
                {
                    std::string object_name = queried_objects[i]->name;
                    std::unique_ptr<ModelData> model_data = std::dynamic_pointer_cast<ModelData>(fillPoseDetails(*queried_objects[i]));
                    if (!model_data) std::cerr << "Failed to cast to ModelData!!! \n" << std::endl;


                    if (object_data_record_[typeid(T).name()].find(object_name) == object_data_record_[typeid(T).name()].end())
                    {
                        // object not found; insert it in map:
                        model_data->unique_id_ = item_id_;
                        // Each item requires two markers hence the item id's for each new item can have even numbers 
                        // and the odd numbers are reserved for the corresponding text labels
                        item_id_ += 2;
                        object_data_record_[typeid(T).name()][object_name] = model_data;
                        if (debug_)
                            std::cout << *queried_objects[i] << std::endl;
                    }
                    else
                    {
                        // object found in map; update its data:
                        model_data->unique_id_ = object_data_record_[typeid(T).name()][object_name]->unique_id_;
                        object_data_record_[typeid(T).name()][object_name] = model_data;
                        if (debug_)
                            std::cout << "Old object data updated in map" << std::endl;
                    }
                }

                for (auto &object_in_map : object_data_record_[typeid(T).name()])
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
                        marker_delete_list_.push_back(object_in_map.second->unique_id_);
                        item_delete_map_[typeid(T).name()].push_back(object_in_map.first);
                        if (debug_)
                            std::cout << "Object in map not found in queried_list. Removing..." << std::endl;
                    }
                }
            }

        private:
            int update_loop_rate_;
            bool debug_;
            int item_id_{0};
            std::string marker_pub_topic_;
            std::string obj_category_mesh_filename_;
            std::string model_config_filename_;
            std::vector<int> marker_delete_list_;
            std::map<std::string, std::vector<std::string>> item_delete_map_;
            std::map<std::string, std::map<std::string, std::unique_ptr<ModelData>>> object_data_record_;
            std::map<std::string, Mesh::Types> obj_category_mesh_map_; 
            std::shared_ptr<ModelLoader> model_loader_;
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
