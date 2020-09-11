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
