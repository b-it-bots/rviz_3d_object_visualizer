#ifndef DATALOADER_MODEL_LOADER
#define DATALOADER_MODEL_LOADER

#include<string>
#include<memory>
#include<vector>

#include <geometry_msgs/Point.h>

#include "dataloader/modelloader/yaml_loader.h"
#include "dataloader/modelloader/model_data.h"

namespace RVizDataLoader
{
    class ModelLoader
    {
    public:
        ModelLoader(const std::string& model_config_path);
        virtual ~ModelLoader(){}

        typedef std::pair<std::unique_ptr<visualization_msgs::Marker>, std::unique_ptr<visualization_msgs::Marker>> MarkerResultPair;

        auto getMeshMarker(int id,
                           Mesh::Types type,
                           const std::string& name,
                           const std::string& frame_id,
                           const std::string& ns,
                           const Utils::Pose<double>& pose = Utils::Pose<double>())
                           -> MarkerResultPair;

        auto getPlaneMarker(int id,
                            const std::string& name,
                            const std::string& frame_id,
                            const std::string& ns,
                            const Utils::Vec3<double>& center,
                            const Utils::Vec3Array<double>& convex_hull,
                            const Utils::Vec4<double>& color = Utils::Vec4<double>(1.0, 0.0, 0.0, 1.0),
                            const Utils::Vec3<double>& scale = Utils::Vec3<double>(1.0, 1.0, 1.0))
                            -> MarkerResultPair;

    protected:
        virtual auto loadModel(Mesh::Types model_type)
                              -> std::unique_ptr<visualization_msgs::Marker>;

        virtual auto getTextLabelMarker(const std::string& name,
                                        const Utils::Vec3<double>& pos) 
                                        -> std::unique_ptr<visualization_msgs::Marker>;

        virtual auto generateTraingleVertices(const Utils::Vec3<double>& center,
                            const Utils::Vec3Array<double>& convex_hull)
                            -> std::vector<geometry_msgs::Point>;

        geometry_msgs::Point asRVizPoint(const Utils::Vec3<double>& point);

        YamlLoader yaml_loader_;
    };
};

#endif // DATALOADER_MODEL_LOADER
