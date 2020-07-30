#ifndef MODEL_LOADER
#define MODEL_LOADER

#include<string>
#include <memory>

#include "dataloader/model_data.h"

namespace RVizDataLoader
{
    class ModelLoader
    {
    public:
        ModelLoader();
        virtual ~ModelLoader(){}

        // TODO: Make this function pure virtual
        virtual std::unique_ptr<ModelData> loadModel(const std::string& model_file_path){}
    };
};

#endif // MODEL_LOADER
