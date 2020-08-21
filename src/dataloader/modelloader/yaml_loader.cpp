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
    mesh_data.mesh_resource_ = node["ResourceFile"].as<std::string>();
    mesh_data.use_color_from_mesh_ = node["UseColorFromMesh"].as<bool>();

    return mesh_data;
}

MeshData YamlLoader::getMeshConfig(Mesh::Types type)
{
    return mesh_data_map_[type];
}
