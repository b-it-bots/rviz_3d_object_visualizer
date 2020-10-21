/*
 * MIT License
 * 
 * Copyright (c) 2020 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/**
  File: mesh.cpp
  Purpose: Class to facilitate easy usage of various mesh types
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

#include "dataloader/modelloader/mesh.h"

using namespace RVizDataLoader;

const std::map<Mesh::Types, std::string> Mesh::MeshTypesMap =  Mesh::initializeMeshTypesMap();

std::map<Mesh::Types, std::string> Mesh::initializeMeshTypesMap()
{
    std::map<Types,std::string> mesh_map;
    mesh_map[Types::BOTTLE] = "BOTTLE";
    mesh_map[Types::CUP] = "CUP";
    mesh_map[Types::PLATE] = "PLATE";
    mesh_map[Types::TABLE] = "TABLE";
    mesh_map[Types::CHAIR] = "CHAIR";
    mesh_map[Types::PERSON] = "PERSON";
    mesh_map[Types::UNKNOWN] = "UNKNOWN";
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
