#ifndef DATALOADER_MODEL_DATA
#define DATALOADER_MODEL_DATA

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "dataloader/modelloader/utils.h"
#include "dataloader/modelloader/model.h"

namespace RVizDataLoader 
{
    class ModelData 
    {
    public:
        ModelData(){}
        virtual ~ModelData(){}

        Utils::Pose<double> pose_;
        Utils::Vec3<double> scale_;
        Utils::Vec3<int> color_;
        std::string mesh_resource_;
        bool use_color_from_mesh_;

        Model::Types type_;
        int unique_id_;
    };
};

#endif // DATALOADER_MODEL_DATA
