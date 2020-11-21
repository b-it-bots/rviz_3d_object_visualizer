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
    ros::init(argc, argv, "ycb_objects");
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

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>(marker_pub_topic_, 1);
    visualization_msgs::MarkerArray msg;
    const int num_ycb_models = 79;

    for (unsigned int row = 0; row < 8; row++)
    {
        for (unsigned int col = 0; col < 10; col++)
        {
            int index = (row * 10) + col;
            if (index >= num_ycb_models)
                break;

            Mesh::Types type = static_cast<Mesh::Types>(Mesh::Types::YCB_002_MASTER_CHEF_CAN + index);
            std::string type_name = Mesh::mesh_types_map_.at(type);
            std::string marker_name = std::string("OBJECT/") + type_name + std::string("/") + type_name;
            auto marker_pair = model_loader.getMeshMarker(index*2, type, marker_name, "base_link", "", 
                                                Utils::Pose<double>(col * 2.0, row, 0.0, 0, 0, 0));
            msg.markers.push_back(*marker_pair.first);
            msg.markers.push_back(*marker_pair.second);
        }
    }

    while (ros::ok()) 
    {
        // Publish the markers
        pub.publish(msg);
        r.sleep();
    }
}
