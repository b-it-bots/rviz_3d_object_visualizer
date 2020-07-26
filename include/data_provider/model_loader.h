#ifndef DATA_PROVIDER
#define DATA_PROVIDER

#include<string>
#include <memory>

#include "data_provider/model_data.h"

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

#endif // DATA_PROVIDER
