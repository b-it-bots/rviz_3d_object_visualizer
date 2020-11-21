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
            BOWL,
            SPOON,
            FORK,
            KNIFE,
            POT,

            // Furniture
            TABLE,
            CHAIR,

            // People
            PERSON,

            // YCB dataset object meshes
            YCB_002_MASTER_CHEF_CAN,
            YCB_003_CRACKER_BOX,
            YCB_004_SUGAR_BOX,
            YCB_005_TOMATO_SOUP_CAN,
            YCB_006_MUSTARD_BOTTLE,
            YCB_007_TUNA_FISH_CAN,
            YCB_008_PUDDING_BOX,
            YCB_009_GELATIN_BOX,
            YCB_010_POTTED_MEAT_CAN,
            YCB_011_BANANA,
            YCB_012_STRAWBERRY,
            YCB_013_APPLE,
            YCB_014_LEMON,
            YCB_015_PEACH,
            YCB_016_PEAR,
            YCB_017_ORANGE,
            YCB_018_PLUM,
            YCB_019_PITCHER_BASE,
            YCB_021_BLEACH_CLEANSER,
            YCB_022_WINDEX_BOTTLE,
            YCB_024_BOWL,
            YCB_025_MUG,
            YCB_026_SPONGE,
            YCB_027_SKILLET,
            YCB_028_SKILLET_LID,
            YCB_029_PLATE,
            YCB_030_FORK,
            YCB_031_SPOON,
            YCB_032_KNIFE,
            YCB_033_SPATULA,
            YCB_035_POWER_DRILL,
            YCB_036_WOOD_BLOCK,
            YCB_037_SCISSORS,
            YCB_038_PADLOCK,
            YCB_040_LARGE_MARKER,
            YCB_042_ADJUSTABLE_WRENCH,
            YCB_043_PHILLIPS_SCREWDRIVER,
            YCB_044_FLAT_SCREWDRIVER,
            YCB_048_HAMMER,
            YCB_050_MEDIUM_CLAMP,
            YCB_051_LARGE_CLAMP,
            YCB_052_EXTRA_LARGE_CLAMP,
            YCB_053_MINI_SOCCER_BALL,
            YCB_054_SOFTBALL,
            YCB_055_BASEBALL,
            YCB_056_TENNIS_BALL,
            YCB_057_RACQUETBALL,
            YCB_058_GOLF_BALL,
            YCB_059_CHAIN,
            YCB_061_FOAM_BRICK,
            YCB_062_DICE,
            YCB_063_A_MARBLES,
            YCB_063_B_MARBLES,
            YCB_065_A_CUPS,
            YCB_065_B_CUPS,
            YCB_065_C_CUPS,
            YCB_065_D_CUPS,
            YCB_065_E_CUPS,
            YCB_065_F_CUPS,
            YCB_065_G_CUPS,
            YCB_065_H_CUPS,
            YCB_065_I_CUPS,
            YCB_065_J_CUPS,
            YCB_070_A_COLORED_WOOD_BLOCKS,
            YCB_070_B_COLORED_WOOD_BLOCKS,
            YCB_071_NINE_HOLE_PEG_TEST,
            YCB_072_A_TOY_AIRPLANE,
            YCB_072_B_TOY_AIRPLANE,
            YCB_072_C_TOY_AIRPLANE,
            YCB_072_D_TOY_AIRPLANE,
            YCB_072_E_TOY_AIRPLANE,
            YCB_073_A_LEGO_DUPLO,
            YCB_073_B_LEGO_DUPLO,
            YCB_073_C_LEGO_DUPLO,
            YCB_073_D_LEGO_DUPLO,
            YCB_073_E_LEGO_DUPLO,
            YCB_073_F_LEGO_DUPLO,
            YCB_073_G_LEGO_DUPLO,
            YCB_077_RUBIKS_CUBE,

            // Others
            BOOK,
            SPRAY_BOTTLE,
            TEDDY_BEAR,
            UNKNOWN,
            SIZE
        };

        static std::map<Types, std::string> initializeMeshTypesMap();
        static const std::map<Types, std::string> mesh_types_map_;

        static Types getMeshType(const std::string& mesh_name);
    };

}

#endif //DATALOADER_MESH
