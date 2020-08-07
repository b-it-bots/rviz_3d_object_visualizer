/**
    File: mongodb_dataloader.h
    Purpose: Loader for MongoDB data
    @author Ahmed Faisal Abdelrahman
    @author Sushant Vijay Chavan
    @version 1.0
*/

#ifndef MONGODB_DATALOADER_H
#define MONGODB_DATALOADER_H

#include <ros/ros.h>
#include <mongodb_store/message_store.h>

namespace RVizDataLoader 
{
    class MongoDBDataloader
    {
        public:
            // Constructor with nh arg required to avoid std::length_error:
            MongoDBDataloader(ros::NodeHandle nh) : message_proxy_(nh) {};
            virtual ~MongoDBDataloader(){};

            virtual void queryDatabase() = 0;
            virtual void runDataUpdateLoop() = 0;
    
        protected:
            mongodb_store::MessageStoreProxy message_proxy_;
    };
}

#endif
