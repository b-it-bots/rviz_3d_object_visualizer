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

#include "dataloader/modelloader/mesh.h"

using namespace RVizDataLoader;

const std::map<Mesh::Types, std::string> Mesh::mesh_types_map_ =  Mesh::initializeMeshTypesMap();

std::map<Mesh::Types, std::string> Mesh::initializeMeshTypesMap()
{
    std::map<Types,std::string> mesh_map;
    mesh_map[Types::BOTTLE] = "BOTTLE";
    mesh_map[Types::CUP] = "CUP";
    mesh_map[Types::PLATE] = "PLATE";
    mesh_map[Types::BOWL] = "BOWL";
    mesh_map[Types::SPOON] = "SPOON";
    mesh_map[Types::FORK] = "FORK";
    mesh_map[Types::KNIFE] = "KNIFE";
    mesh_map[Types::POT] = "POT";
    mesh_map[Types::TABLE] = "TABLE";
    mesh_map[Types::CHAIR] = "CHAIR";
    mesh_map[Types::PERSON] = "PERSON";

    mesh_map[Types::YCB_002_MASTER_CHEF_CAN] = "YCB_002_MASTER_CHEF_CAN";
    mesh_map[Types::YCB_003_CRACKER_BOX] = "YCB_003_CRACKER_BOX";
    mesh_map[Types::YCB_004_SUGAR_BOX] = "YCB_004_SUGAR_BOX";
    mesh_map[Types::YCB_005_TOMATO_SOUP_CAN] = "YCB_005_TOMATO_SOUP_CAN";
    mesh_map[Types::YCB_006_MUSTARD_BOTTLE] = "YCB_006_MUSTARD_BOTTLE";
    mesh_map[Types::YCB_007_TUNA_FISH_CAN] = "YCB_007_TUNA_FISH_CAN";
    mesh_map[Types::YCB_008_PUDDING_BOX] = "YCB_008_PUDDING_BOX";
    mesh_map[Types::YCB_009_GELATIN_BOX] = "YCB_009_GELATIN_BOX";
    mesh_map[Types::YCB_010_POTTED_MEAT_CAN] = "YCB_010_POTTED_MEAT_CAN";
    mesh_map[Types::YCB_011_BANANA] = "YCB_011_BANANA";
    mesh_map[Types::YCB_012_STRAWBERRY] = "YCB_012_STRAWBERRY";
    mesh_map[Types::YCB_013_APPLE] = "YCB_013_APPLE";
    mesh_map[Types::YCB_014_LEMON] = "YCB_014_LEMON";
    mesh_map[Types::YCB_015_PEACH] = "YCB_015_PEACH";
    mesh_map[Types::YCB_016_PEAR] = "YCB_016_PEAR";
    mesh_map[Types::YCB_017_ORANGE] = "YCB_017_ORANGE";
    mesh_map[Types::YCB_018_PLUM] = "YCB_018_PLUM";
    mesh_map[Types::YCB_019_PITCHER_BASE] = "YCB_019_PITCHER_BASE";
    mesh_map[Types::YCB_021_BLEACH_CLEANSER] = "YCB_021_BLEACH_CLEANSER";
    mesh_map[Types::YCB_022_WINDEX_BOTTLE] = "YCB_022_WINDEX_BOTTLE";
    mesh_map[Types::YCB_024_BOWL] = "YCB_024_BOWL";
    mesh_map[Types::YCB_025_MUG] = "YCB_025_MUG";
    mesh_map[Types::YCB_026_SPONGE] = "YCB_026_SPONGE";
    mesh_map[Types::YCB_027_SKILLET] = "YCB_027_SKILLET";
    mesh_map[Types::YCB_028_SKILLET_LID] = "YCB_028_SKILLET_LID";
    mesh_map[Types::YCB_029_PLATE] = "YCB_029_PLATE";
    mesh_map[Types::YCB_030_FORK] = "YCB_030_FORK";
    mesh_map[Types::YCB_031_SPOON] = "YCB_031_SPOON";
    mesh_map[Types::YCB_032_KNIFE] = "YCB_032_KNIFE";
    mesh_map[Types::YCB_033_SPATULA] = "YCB_033_SPATULA";
    mesh_map[Types::YCB_035_POWER_DRILL] = "YCB_035_POWER_DRILL";
    mesh_map[Types::YCB_036_WOOD_BLOCK] = "YCB_036_WOOD_BLOCK";
    mesh_map[Types::YCB_037_SCISSORS] = "YCB_037_SCISSORS";
    mesh_map[Types::YCB_038_PADLOCK] = "YCB_038_PADLOCK";
    mesh_map[Types::YCB_040_LARGE_MARKER] = "YCB_040_LARGE_MARKER";
    mesh_map[Types::YCB_042_ADJUSTABLE_WRENCH] = "YCB_042_ADJUSTABLE_WRENCH";
    mesh_map[Types::YCB_043_PHILLIPS_SCREWDRIVER] = "YCB_043_PHILLIPS_SCREWDRIVER";
    mesh_map[Types::YCB_044_FLAT_SCREWDRIVER] = "YCB_044_FLAT_SCREWDRIVER";
    mesh_map[Types::YCB_048_HAMMER] = "YCB_048_HAMMER";
    mesh_map[Types::YCB_050_MEDIUM_CLAMP] = "YCB_050_MEDIUM_CLAMP";
    mesh_map[Types::YCB_051_LARGE_CLAMP] = "YCB_051_LARGE_CLAMP";
    mesh_map[Types::YCB_052_EXTRA_LARGE_CLAMP] = "YCB_052_EXTRA_LARGE_CLAMP";
    mesh_map[Types::YCB_053_MINI_SOCCER_BALL] = "YCB_053_MINI_SOCCER_BALL";
    mesh_map[Types::YCB_054_SOFTBALL] = "YCB_054_SOFTBALL";
    mesh_map[Types::YCB_055_BASEBALL] = "YCB_055_BASEBALL";
    mesh_map[Types::YCB_056_TENNIS_BALL] = "YCB_056_TENNIS_BALL";
    mesh_map[Types::YCB_057_RACQUETBALL] = "YCB_057_RACQUETBALL";
    mesh_map[Types::YCB_058_GOLF_BALL] = "YCB_058_GOLF_BALL";
    mesh_map[Types::YCB_059_CHAIN] = "YCB_059_CHAIN";
    mesh_map[Types::YCB_061_FOAM_BRICK] = "YCB_061_FOAM_BRICK";
    mesh_map[Types::YCB_062_DICE] = "YCB_062_DICE";
    mesh_map[Types::YCB_063_A_MARBLES] = "YCB_063_A_MARBLES";
    mesh_map[Types::YCB_063_B_MARBLES] = "YCB_063_B_MARBLES";
    mesh_map[Types::YCB_065_A_CUPS] = "YCB_065_A_CUPS";
    mesh_map[Types::YCB_065_B_CUPS] = "YCB_065_B_CUPS";
    mesh_map[Types::YCB_065_C_CUPS] = "YCB_065_C_CUPS";
    mesh_map[Types::YCB_065_D_CUPS] = "YCB_065_D_CUPS";
    mesh_map[Types::YCB_065_E_CUPS] = "YCB_065_E_CUPS";
    mesh_map[Types::YCB_065_F_CUPS] = "YCB_065_F_CUPS";
    mesh_map[Types::YCB_065_G_CUPS] = "YCB_065_G_CUPS";
    mesh_map[Types::YCB_065_H_CUPS] = "YCB_065_H_CUPS";
    mesh_map[Types::YCB_065_I_CUPS] = "YCB_065_I_CUPS";
    mesh_map[Types::YCB_065_J_CUPS] = "YCB_065_J_CUPS";
    mesh_map[Types::YCB_070_A_COLORED_WOOD_BLOCKS] = "YCB_070_A_COLORED_WOOD_BLOCKS";
    mesh_map[Types::YCB_070_B_COLORED_WOOD_BLOCKS] = "YCB_070_B_COLORED_WOOD_BLOCKS";
    mesh_map[Types::YCB_071_NINE_HOLE_PEG_TEST] = "YCB_071_NINE_HOLE_PEG_TEST";
    mesh_map[Types::YCB_072_A_TOY_AIRPLANE] = "YCB_072_A_TOY_AIRPLANE";
    mesh_map[Types::YCB_072_B_TOY_AIRPLANE] = "YCB_072_B_TOY_AIRPLANE";
    mesh_map[Types::YCB_072_C_TOY_AIRPLANE] = "YCB_072_C_TOY_AIRPLANE";
    mesh_map[Types::YCB_072_D_TOY_AIRPLANE] = "YCB_072_D_TOY_AIRPLANE";
    mesh_map[Types::YCB_072_E_TOY_AIRPLANE] = "YCB_072_E_TOY_AIRPLANE";
    mesh_map[Types::YCB_073_A_LEGO_DUPLO] = "YCB_073_A_LEGO_DUPLO";
    mesh_map[Types::YCB_073_B_LEGO_DUPLO] = "YCB_073_B_LEGO_DUPLO";
    mesh_map[Types::YCB_073_C_LEGO_DUPLO] = "YCB_073_C_LEGO_DUPLO";
    mesh_map[Types::YCB_073_D_LEGO_DUPLO] = "YCB_073_D_LEGO_DUPLO";
    mesh_map[Types::YCB_073_E_LEGO_DUPLO] = "YCB_073_E_LEGO_DUPLO";
    mesh_map[Types::YCB_073_F_LEGO_DUPLO] = "YCB_073_F_LEGO_DUPLO";
    mesh_map[Types::YCB_073_G_LEGO_DUPLO] = "YCB_073_G_LEGO_DUPLO";
    mesh_map[Types::YCB_077_RUBIKS_CUBE] = "YCB_077_RUBIKS_CUBE";

    mesh_map[Types::BOOK] = "BOOK";
    mesh_map[Types::TEDDY_BEAR] = "TEDDY_BEAR";
    mesh_map[Types::SPRAY_BOTTLE] = "SPRAY_BOTTLE";
    mesh_map[Types::UNKNOWN] = "UNKNOWN";
    return mesh_map;
}

Mesh::Types Mesh::getMeshType(const std::string& mesh_name)
{
    Types type = Types::INVALID;
    for (const auto& m: mesh_types_map_)
    {
        if (m.second == mesh_name)
        {
            type = m.first;
            break;
        }
    }

    return type;
}
