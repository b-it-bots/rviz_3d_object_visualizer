/**
    File: abstract_dataloader.h
    Purpose: Abstract loader for MongoDB data
    @author Ahmed Faisal Abdelrahman
    @author Sushant Vijay Chavan
    @version 1.0
*/

#ifndef ABSTRACT_DATALOADER_H
#define ABSTRACT_DATALOADER_H

#include <ros/ros.h>
#include <mongodb_store/message_store.h>

namespace RVizDataLoader 
{
    class AbstractDataloader
    {
        public:
            // Constructor with nh arg required to avoid std::length_error:
            AbstractDataloader(ros::NodeHandle nh) : message_proxy_(nh) {};
            virtual ~AbstractDataloader(){};

            virtual void queryDatabase() = 0;
            virtual void runDataUpdateLoop() = 0;
            virtual void publishObjectData() = 0;
    
        protected:
            mongodb_store::MessageStoreProxy message_proxy_;
            ros::Publisher data_pub_;
    };
}

#endif
