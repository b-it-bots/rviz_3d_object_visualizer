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

#ifndef DATALOADER_MODEL_DATA
#define DATALOADER_MODEL_DATA

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "dataloader/modelloader/utils.h"
#include "dataloader/modelloader/mesh.h"

namespace RVizDataLoader 
{
    struct ModelData
    {
        ModelData(){}

        ModelData(const ModelData& data):
        scale_(data.scale_),
        color_(data.color_),
        text_offset_(data.text_offset_),
        unique_id_(data.unique_id_),
        name_(data.name_){}

        virtual ~ModelData(){}

        Utils::Vec3<double> scale_;
        Utils::Vec3<int> color_;
        Utils::Vec3<double> text_offset_;
        int unique_id_;
        std::string name_;
    };

    struct PlaneData : public ModelData
    {
        PlaneData(){}

        PlaneData(const PlaneData& data):
        ModelData(data),
        center_(data.center_),
        convex_hull_(data.convex_hull_){}

        virtual ~PlaneData(){}

        Utils::Vec3<double> center_;
        Utils::Vec3Array<double> convex_hull_;
    };

    struct MeshData : public ModelData
    {
        MeshData(){}

        MeshData(const MeshData& data):
        ModelData(data),
        pose_(data.pose_),
        mesh_resource_(data.mesh_resource_),
        use_color_from_mesh_(data.use_color_from_mesh_),
        type_(data.type_){}

        virtual ~MeshData(){}

        Utils::Pose<double> pose_;
        std::string mesh_resource_;
        bool use_color_from_mesh_;

        Mesh::Types type_;
    };
};

#endif // DATALOADER_MODEL_DATA
