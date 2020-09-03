#include "dataloader/modelloader/mesh.h"

using namespace RVizDataLoader;

const std::map<Mesh::Types, std::string> Mesh::MeshTypesMap =  Mesh::initializeMeshTypesMap();

std::map<Mesh::Types, std::string> Mesh::initializeMeshTypesMap()
{
    std::map<Types,std::string> mesh_map;
    mesh_map[Types::BOTTLE] = "BOTTLE";
    mesh_map[Types::TABLE] = "TABLE";
    mesh_map[Types::CHAIR] = "CHAIR";
    mesh_map[Types::PERSON] = "PERSON";
    return mesh_map;
}

Mesh::Types Mesh::getMeshType(const std::string& mesh_name)
{
    Types type = Types::INVALID;
    for (const auto& m: MeshTypesMap)
    {
        if (m.second == mesh_name)
        {
            type = m.first;
            break;
        }
    }

    return type;
}
