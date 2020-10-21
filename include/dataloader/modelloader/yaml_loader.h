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
  File: yaml_loader.h
  Purpose: Utility class to load mesh configurations from a YAML file.
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

#ifndef DATALOADER_YAML_LOADER
#define DATALOADER_YAML_LOADER

#include <map>
#include <memory>

#include <yaml-cpp/yaml.h>

#include "dataloader/modelloader/model_data.h"

namespace RVizDataLoader 
{
    class YamlLoader 
    {
    public:
        YamlLoader(const std::string& yaml_file);
        virtual ~YamlLoader(){}

        auto getMeshConfig(Mesh::Types type) -> std::unique_ptr<MeshData>;
        void setYamlFilePath(const std::string& filepath);

    protected:
        MeshData extractMeshConfig(YAML::Node node);
        void loadYamlFile();

        std::string yaml_file_path_;
        std::map<Mesh::Types, MeshData> mesh_data_map_;

    };
};

#endif //DATALOADER_YAML_LOADER
