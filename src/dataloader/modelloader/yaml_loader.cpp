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

        // since the config file has changed, reload the model configurations
        model_data_map_.clear();
        loadYamlFile();
    }
}

void YamlLoader::loadYamlFile()
{
    if (yaml_file_path_ == "")
    {
        std::cout << "[YAML_LOADER] Error: Empty Yaml file! Could not load model configurations" << std::endl;
        return;
    }

    YAML::Node root_node = YAML::LoadFile(yaml_file_path_);
    std::cout << "Loaded YAML file" << std::endl;

    for (const auto& node: root_node)
    {
        Model::Types type = Model::getType(node.first.as<std::string>());
        ModelData data = extractModelConfig(node.second);
        model_data_map_.insert(std::pair<Model::Types, ModelData>(type, data));
    }
}

ModelData YamlLoader::extractModelConfig(YAML::Node node)
{
    ModelData model_data;

    model_data.pose_.update(node["Position"].as<std::vector<double>>(),
                            node["Orientation"].as<std::vector<double>>());
    model_data.scale_.update(node["Scale"].as<std::vector<double>>());
    model_data.color_.update(node["Color"].as<std::vector<int>>());
    model_data.mesh_resource_ = node["ResourceFile"].as<std::string>();
    model_data.use_color_from_mesh_ = node["UseColorFromMesh"].as<bool>();

    return model_data;
}

ModelData YamlLoader::getModelConfig(Model::Types type)
{
    return model_data_map_[type];
}
