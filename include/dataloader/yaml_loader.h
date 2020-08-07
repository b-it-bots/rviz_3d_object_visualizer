#ifndef YAML_LOADER
#define YAML_LOADER

#include <map>
#include <memory>

#include <yaml-cpp/yaml.h>

#include "dataloader/model_data.h"

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
        void initializeModelIdMap();
        ModelData extractModelConfig(YAML::Node node);
        void loadYamlFile();

        std::string yaml_file_path_;
        std::map<Model::Types, ModelData> model_data_map_;

    };
};

#endif //YAML_LOADER
