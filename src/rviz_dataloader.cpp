#include <ros/ros.h>

#include "dataloader/mdr_dataloader.h"

using namespace RVizDataLoader;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_dataloader");
    ros::NodeHandle nh;

    MDRDataloader mdr_dataloader = MDRDataloader(nh);
    /* mdr_dataloader.queryDatabase(); */
    /* mdr_dataloader.publishObjectData(); */

    mdr_dataloader.runDataUpdateLoop();

    return 0;
}
