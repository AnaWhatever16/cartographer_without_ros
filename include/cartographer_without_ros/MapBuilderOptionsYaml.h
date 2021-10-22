#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;

/// This class is in charge of setting all the parameters of the MapBuilder, given the path where all the config files should exist.
class MapBuilderOptionsYaml{
    public:
        /// Constructor.
        /// \param _path path to folder of config files
        MapBuilderOptionsYaml(std::string _path);

        /// This method will return the configuration for the MapBuilder that we desire.
        mapping::proto::MapBuilderOptions getMapOpt() { return mapBuilderOpt_; }

    private:
        void setMapBuilderOptions(std::string _path);
        void setPoseGraphOptions(YAML::Node _config);

    private:
        mapping::proto::MapBuilderOptions                                       mapBuilderOpt_;
        mapping::proto::PoseGraphOptions                                        poseGraphOpt_;

        mapping::optimization::proto::OptimizationProblemOptions                optimProblemOpt_;
        common::proto::CeresSolverOptions                                       ceresSolverOptimProblemOpt_;
        mapping::constraints::proto::ConstraintBuilderOptions                   constraintOpt_;

        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D      fastCorrScan2DOpt_;
        mapping::scan_matching::proto::CeresScanMatcherOptions2D                ceresScan2DOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver2DOpt_;

        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions3D      fastCorrScan3DOpt;
        mapping::scan_matching::proto::CeresScanMatcherOptions3D                ceresScan3DOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver3DOpt_;

};

