/**
    File: mdr_dataloader.h
    Purpose: Loader for b-it-bots MAS MongoDB data
    @author Ahmed Faisal Abdelrahman
    @author Sushant Vijay Chavan
    @version 1.0
*/

#ifndef MDR_DATALOADER_H
#define MDR_DATALOADER_H

#include "dataloader/mongodb_dataloader.h"

namespace RVizDataLoader 
{
    class MDRDataloader: public MongoDBDataloader
    {
        public:
            MDRDataloader();
            virtual ~MDRDataloader();

            virtual void queryDatabase();
            virtual void runDataUpdateLoop();
    
        private:
    };
}

#endif
