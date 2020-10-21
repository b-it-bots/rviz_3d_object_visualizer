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
  File: mesh.h
  Purpose: Class to facilitate easy usage of various mesh types
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

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
