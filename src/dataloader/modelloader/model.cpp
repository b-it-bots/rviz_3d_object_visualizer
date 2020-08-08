#include "dataloader/modelloader/model.h"

using namespace RVizDataLoader;

const std::map<Model::Types, std::string> Model::ModelTypesMap =  Model::initializeModelMap();

std::map<Model::Types, std::string> Model::initializeModelMap()
{
    std::map<Types,std::string> model_map;
    model_map[Types::BOTTLE] = "BOTTLE";
    model_map[Types::TABLE] = "TABLE";
    model_map[Types::CHAIR] = "CHAIR";
    model_map[Types::PERSON] = "PERSON";
    return model_map;
}

Model::Types Model::getType(const std::string& model_name)
{
    Types type = Types::INVALID;
    for (const auto& m: ModelTypesMap)
    {
        if (m.second == model_name)
        {
            type = m.first;
            break;
        }
    }

    return type;
}
