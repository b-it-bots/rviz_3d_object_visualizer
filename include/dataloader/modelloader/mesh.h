#ifndef DATALOADER_MESH
#define DATALOADER_MESH

#include <string>
#include <map>

namespace RVizDataLoader 
{

class Mesh
{
public:
    enum Types 
    {
        INVALID = 0,
        // Kitchen items
        BOTTLE,
        CUP,
        PLATE,

        // Furniture
        TABLE,
        CHAIR,

        // People
        PERSON,

        // Others
        UNKNOWN,
        SIZE
    };

    static std::map<Types, std::string> initializeMeshTypesMap();
    static const std::map<Types, std::string> MeshTypesMap;

    static Types getMeshType(const std::string& mesh_name);
};

}

#endif //DATALOADER_MESH
