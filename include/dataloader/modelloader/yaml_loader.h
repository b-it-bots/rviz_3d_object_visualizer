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

        ModelData getModelConfig(Model::Types type);
        void setYamlFilePath(const std::string& filepath);

    protected:
        ModelData extractModelConfig(YAML::Node node);
        void loadYamlFile();

        std::string yaml_file_path_;
        std::map<Model::Types, ModelData> model_data_map_;

    };
};

#endif //DATALOADER_YAML_LOADER
