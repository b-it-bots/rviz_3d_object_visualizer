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
        unique_id_(data.unique_id_){}

        virtual ~ModelData(){}

        Utils::Vec3<double> scale_;
        Utils::Vec3<int> color_;
        int unique_id_;
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
