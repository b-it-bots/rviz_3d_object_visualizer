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
        virtual ~ModelData(){}

        Utils::Vec3<double> scale_;
        Utils::Vec3<int> color_;
        int unique_id_;
    };

    struct PlaneData : public ModelData
    {
        PlaneData(){}
        virtual ~PlaneData(){}

        Utils::Vec3<double> center_;
        Utils::Vec3Array<double> convex_hull_;
    };

    struct MeshData : public ModelData
    {
        MeshData(){}
        virtual ~MeshData(){}

        Utils::Pose<double> pose_;
        std::string mesh_resource_;
        bool use_color_from_mesh_;

        Mesh::Types type_;
    };
};

#endif // DATALOADER_MODEL_DATA
