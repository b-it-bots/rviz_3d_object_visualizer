/**
     File: dataloader.cpp
     Purpose: Launches data loading node
     @author Ahmed Faisal Abdelrahman
     @author Sushant Vijay Chavan
     @version 1.0
 */


#include <ros/ros.h>

#include "dataloader/mdr_dataloader.h"

using namespace RVizDataLoader;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataloader_node");
    ros::NodeHandle nh;

    MDRDataloader mdr_dataloader = MDRDataloader(nh);
    /* mdr_dataloader.queryDatabase(); */
    /* mdr_dataloader.publishObjectData(); */

    mdr_dataloader.runDataUpdateLoop();

    return 0;
}
