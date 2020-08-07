#ifndef DATALOADER_MODEL
#define DATALOADER_MODEL

#include <string>
#include <map>

namespace RVizDataLoader 
{
    class Model
    {
    public:
        enum Types 
        {
            INVALID = 0,
            BOTTLE,
            TABLE,
            CHAIR,
            PERSON,
            SIZE
        };

        static std::map<Types, std::string> initializeModelMap();
        static const std::map<Types, std::string> ModelTypesMap;

        static Types getType(const std::string& model_name);
};

}

#endif //DATALOADER_MODEL
