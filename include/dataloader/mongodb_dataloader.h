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
            MongoDBDataloader();
            virtual ~MongoDBDataloader();
    
        private:
            ros::NodeHandle nh_;
            mongodb_store::MessageStoreProxy message_proxy_;
    };
}

#endif
