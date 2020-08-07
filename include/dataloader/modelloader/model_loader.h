#ifndef DATALOADER_MODEL_LOADER
#define DATALOADER_MODEL_LOADER

#include<string>
#include<memory>

#include "dataloader/modelloader/yaml_loader.h"
#include "dataloader/modelloader/model_data.h"

namespace RVizDataLoader
{
    class ModelLoader
    {
    public:
        ModelLoader(const std::string& model_config_path);
        virtual ~ModelLoader(){}

        auto getMarker(int id,
                       Model::Types type, 
                       const std::string& frame_id, 
                       const std::string& ns)
                       -> std::unique_ptr<visualization_msgs::Marker>;

    protected:
        virtual auto loadModel(Model::Types model_type)
                              -> std::unique_ptr<visualization_msgs::Marker>;

        YamlLoader yaml_loader_;
    };
};

#endif // DATALOADER_MODEL_LOADER
