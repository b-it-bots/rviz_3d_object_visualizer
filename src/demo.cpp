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

#include <iostream>
#include <typeinfo>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "dataloader/modelloader/model_loader.h"

using namespace RVizDataLoader;

int main(int argc, char** argv)
{
    // Initialize the ros node
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);

    // Get the path to the config file storing the model params
    std::string mesh_config_file;
    if (!ros::param::get("/demo_node/model_config_filename", mesh_config_file)) //ros::param::get("relative_name", relative_name)
    {
        std::cout << "Failed to get the param: model_config_filename" << std::endl;
        mesh_config_file = "model_params.yaml";
    }
    mesh_config_file = ros::package::getPath("rviz_3d_object_visualizer") + "/config/" + mesh_config_file;

    // Get the topic to publish the markers
    std::string marker_pub_topic_;
    if (!ros::param::get("/demo_node/marker_pub_topic", marker_pub_topic_))
    {
        std::cout << "Failed to get the param: marker_pub_topic" << std::endl;
        marker_pub_topic_ = "/rviz_3d_object_visualizer/markers";
    }

    // Create a model loader
    ModelLoader model_loader = ModelLoader(mesh_config_file);

    // Create few demo 3d objects using the model loader. 
    // Notice that the markers can only have even numbered ids.
    auto bottle_marker_pair = model_loader.getMeshMarker(0, Mesh::Types::BOTTLE, "OBJECT/BOTTLE/coke", "base_link", "", 
                                                Utils::Pose<double>(0.0, 0.5, 0.78, 0, 0, 0));
    auto table_marker_pair = model_loader.getMeshMarker(2, Mesh::Types::TABLE, "OBJECT/TABLE/my_table", "base_link", "");
    auto chair_marker_pair = model_loader.getMeshMarker(4, Mesh::Types::CHAIR, "OBJECT/CHAIR/my_chair", "base_link", "", 
                                                Utils::Pose<double>(-0.25, 0, 0, 0, 0, 0));
    auto person_marker_pair = model_loader.getMeshMarker(8, Mesh::Types::PERSON, "PERSON/Sophia", "base_link", "", 
                                                Utils::Pose<double>(1.0, 0, 0, 0, 0, 3.14));
    auto cup_marker_pair = model_loader.getMeshMarker(10, Mesh::Types::CUP, "OBJECT/CUP/noodle_cup", "base_link", "", 
                                                Utils::Pose<double>(0.0, -0.5, 0.78, 0, 0, 0));

    // Create a plane object marker
    Utils::Vec3Array<double> plane_vert;
    plane_vert.push_back(Utils::Vec3<double>(-1, 1, 0.0));
    plane_vert.push_back(Utils::Vec3<double>(1, 1, 0.0));
    plane_vert.push_back(Utils::Vec3<double>(1, -1, 0.0));
    plane_vert.push_back(Utils::Vec3<double>(-1, -1, 0.0));
    auto plane_marker_pair = model_loader.getPlaneMarker(12, "PLANE/carpet", "base_link", "", Utils::Vec3<double>(0, 0, 0.0), plane_vert);

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>(marker_pub_topic_, 1);
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(*bottle_marker_pair.first);
    msg.markers.push_back(*cup_marker_pair.first);
    msg.markers.push_back(*table_marker_pair.first);
    msg.markers.push_back(*chair_marker_pair.first);
    msg.markers.push_back(*person_marker_pair.first);
    msg.markers.push_back(*bottle_marker_pair.second);
    msg.markers.push_back(*cup_marker_pair.second);
    msg.markers.push_back(*table_marker_pair.second);
    msg.markers.push_back(*chair_marker_pair.second);
    msg.markers.push_back(*person_marker_pair.second);
    msg.markers.push_back(*plane_marker_pair.first);
    msg.markers.push_back(*plane_marker_pair.second);

    while (ros::ok()) 
    {
        // Publish the markers
        pub.publish(msg);
        r.sleep();
    }
}
