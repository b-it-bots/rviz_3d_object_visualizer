#include <iostream>
#include "dataloader/model_loader.h"
#include "dataloader/mdr_dataloader.h"

int main()
{
    RVizDataLoader::ModelLoader model_loader = RVizDataLoader::ModelLoader();
    RVizDataLoader::MDRDataloader mongodb_dataloader = RVizDataLoader::MDRDataloader();
    std::cout << "Exiting..." << std::endl;
}
