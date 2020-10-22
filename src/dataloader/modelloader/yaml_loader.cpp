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

#include "dataloader/modelloader/yaml_loader.h"

using namespace RVizDataLoader;

YamlLoader::YamlLoader(const std::string& yaml_file)
: yaml_file_path_(yaml_file)
{
    loadYamlFile();
}

void YamlLoader::setYamlFilePath(const std::string& filepath)
{
    if (filepath != yaml_file_path_)
    {
        yaml_file_path_ = filepath;

        // since the config file has changed, reload the mesh configurations
        mesh_data_map_.clear();
        loadYamlFile();
    }
}

void YamlLoader::loadYamlFile()
{
    if (yaml_file_path_ == "")
    {
        std::cout << "[YAML_LOADER] Error: Empty Yaml file! Could not load mesh configurations" << std::endl;
        return;
    }

    YAML::Node root_node = YAML::LoadFile(yaml_file_path_);
    for (const auto& node: root_node)
    {
        Mesh::Types type = Mesh::getMeshType(node.first.as<std::string>());
        MeshData data = extractMeshConfig(node.second);
        mesh_data_map_.insert(std::pair<Mesh::Types, MeshData>(type, data));
    }
}

MeshData YamlLoader::extractMeshConfig(YAML::Node node)
{
    MeshData mesh_data;

    mesh_data.pose_.update(node["Position"].as<std::vector<double>>(),
                            node["Orientation"].as<std::vector<double>>());
    mesh_data.scale_.update(node["Scale"].as<std::vector<double>>());
    mesh_data.color_.update(node["Color"].as<std::vector<int>>());
    mesh_data.text_offset_.update(node["TextOffset"].as<std::vector<double>>());
    mesh_data.mesh_resource_ = node["ResourceFile"].as<std::string>();
    mesh_data.use_color_from_mesh_ = node["UseColorFromMesh"].as<bool>();

    return mesh_data;
}

auto YamlLoader::getMeshConfig(Mesh::Types type) -> std::unique_ptr<MeshData>
{
    std::unique_ptr<MeshData> mesh_data;
    std::map<Mesh::Types, MeshData>::iterator itr = mesh_data_map_.find(type);
    if (itr != mesh_data_map_.end())
    {
        mesh_data = std::unique_ptr<MeshData>(new MeshData(itr->second));
    }
    else
    {
        mesh_data = std::unique_ptr<MeshData>(new MeshData(mesh_data_map_.at(Mesh::UNKNOWN)));
    }


    return mesh_data;
}
