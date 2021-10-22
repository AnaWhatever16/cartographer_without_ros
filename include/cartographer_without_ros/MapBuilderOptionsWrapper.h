#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <cartographer_without_ros/TrajectoryBuilderOptionsWrapper.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;
class MapBuilderOptionsWrapper{
    public:
        MapBuilderOptionsWrapper(YAML::Node _config);
        mapping::proto::MapBuilderOptions getMapOpt() { return mapBuilderOpt; }


    private:
        void setMapBuilderOptions(YAML::Node _config);
        void setPoseGraphOptions(YAML::Node _config);

    private:
        mapping::proto::MapBuilderOptions                                       mapBuilderOpt;

        mapping::proto::PoseGraphOptions                                        poseGraphOpt;
        mapping::constraints::proto::ConstraintBuilderOptions                   constraintOpt;
        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D      fastCorrScan2DOpt;
        mapping::scan_matching::proto::CeresScanMatcherOptions2D                ceresScan2DPoseGraphOpt;
        common::proto::CeresSolverOptions                                       ceresSolverOpt2DPoseGraph;
        mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions3D      fastCorrScan3DOpt;
        mapping::scan_matching::proto::CeresScanMatcherOptions3D                ceresScan3DPoseGraphOpt;
        common::proto::CeresSolverOptions                                       ceresSolverOpt3DPoseGraph;
        mapping::optimization::proto::OptimizationProblemOptions                optimizationProblemOpt;
        common::proto::CeresSolverOptions                                       ceresSolverOptOptimizationProblem;
};

