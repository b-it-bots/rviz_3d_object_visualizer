/**
     File: mongodb_dataloader.cpp
     Purpose: ...
     @author Ahmed Faisal Abdelrahman
     @author Sushant Vijay Chavan
     @version 1.0
 */

#include "dataloader/mongodb_dataloader.h"

using namespace RVizDataLoader;

MongoDBDataloader::MongoDBDataloader()
    : message_proxy_(nh_)
{
}

MongoDBDataloader::~MongoDBDataloader()
{
}
