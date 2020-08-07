#ifndef MODEL_DATA
#define MODEL_DATA

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "dataloader/utils.h"
#include "dataloader/model.h"

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
    };
};

#endif // MODEL_DATA
