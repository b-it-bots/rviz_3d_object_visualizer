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

/**
  File: model_loader.h
  Purpose: Generates 3D RViz markers for the mesh and its text label from a mesh type and pose
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

#ifndef DATALOADER_MODEL_LOADER
#define DATALOADER_MODEL_LOADER

#include<string>
#include<memory>
#include<vector>

#include <geometry_msgs/Point.h>

#include "dataloader/modelloader/yaml_loader.h"
#include "dataloader/modelloader/model_data.h"

namespace RVizDataLoader
{
    class ModelLoader
    {
    public:
        ModelLoader(const std::string& model_config_path);
        virtual ~ModelLoader(){}

        typedef std::pair<std::unique_ptr<visualization_msgs::Marker>, std::unique_ptr<visualization_msgs::Marker>> MarkerResultPair;

        auto getMeshMarker(int id,
                           Mesh::Types type,
                           const std::string& name,
                           const std::string& frame_id,
                           const std::string& ns,
                           const Utils::Pose<double>& pose = Utils::Pose<double>())
                           -> MarkerResultPair;

        auto getPlaneMarker(int id,
                            const std::string& name,
                            const std::string& frame_id,
                            const std::string& ns,
                            const Utils::Vec3<double>& center,
                            const Utils::Vec3Array<double>& convex_hull,
                            const Utils::Vec4<double>& color = Utils::Vec4<double>(1.0, 0.0, 0.0, 1.0),
                            const Utils::Vec3<double>& scale = Utils::Vec3<double>(1.0, 1.0, 1.0))
                            -> MarkerResultPair;

    protected:
        virtual auto loadModel(Mesh::Types model_type)
                              -> std::unique_ptr<visualization_msgs::Marker>;

        virtual auto getTextLabelMarker(const std::string& name,
                                        const Utils::Vec3<double>& pos) 
                                        -> std::unique_ptr<visualization_msgs::Marker>;

        virtual auto generateTraingleVertices(const Utils::Vec3<double>& center,
                            const Utils::Vec3Array<double>& convex_hull)
                            -> std::vector<geometry_msgs::Point>;

        geometry_msgs::Point asRVizPoint(const Utils::Vec3<double>& point);

        YamlLoader yaml_loader_;
    };
};

#endif // DATALOADER_MODEL_LOADER
