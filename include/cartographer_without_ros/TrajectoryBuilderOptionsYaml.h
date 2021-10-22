#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;
class TrajectoryBuilderOptionsYaml{
    public:
        TrajectoryBuilderOptionsYaml(YAML::Node _config);
        mapping::proto::TrajectoryBuilderOptions getTrajectoryOpt() { return trajBuilderOpt; }


    private:
        void setTrajectoryBuilder2DOptions(YAML::Node _config);
        void setTrajectoryBuilder3DOptions(YAML::Node _config);
        void setTrajectoryBuilderOptions(YAML::Node _config);

    private:
        mapping::proto::TrajectoryBuilderOptions                                trajBuilderOpt;

        mapping::proto::LocalTrajectoryBuilderOptions2D                         traj2dOpt;
        sensor::proto::AdaptiveVoxelFilterOptions                               adaptVoxelOpt;
        sensor::proto::AdaptiveVoxelFilterOptions                               loopClosureAdaptVoxelOpt;
        mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions    realTimeCorrOpt2D;
        mapping::scan_matching::proto::CeresScanMatcherOptions2D                ceresScan2DTrajOpt;
        common::proto::CeresSolverOptions                                       ceresSolverOpt2DTraj;
        mapping::proto::MotionFilterOptions                                     motionFilter2dOpt;
        mapping::proto::PoseExtrapolatorOptions                                 poseExtrapolator2dOpt;
        mapping::proto::ConstantVelocityPoseExtrapolatorOptions                 constVel2dOpt;
        mapping::proto::ImuBasedPoseExtrapolatorOptions                         imuBasedPoseExtrap2dOpt;
        common::proto::CeresSolverOptions                                       solverOptImuBased2d;
        mapping::proto::SubmapsOptions2D                                        submapsOpt2d;
        mapping::proto::GridOptions2D                                           gridOpt2d;
        mapping::proto::RangeDataInserterOptions                                rangeDataInserterOpt2d;
        mapping::proto::ProbabilityGridRangeDataInserterOptions2D               probGridRangeDataInsert2DOpt;
        mapping::proto::TSDFRangeDataInserterOptions2D                          tsdfRangeInsertOpt2D;
        mapping::proto::NormalEstimationOptions2D                               normalEstimation2DOpt;

        mapping::proto::LocalTrajectoryBuilderOptions3D                         traj3dOpt;
        sensor::proto::AdaptiveVoxelFilterOptions                               highResAdaptVoxelOpt;
        sensor::proto::AdaptiveVoxelFilterOptions                               lowResAdaptVoxelOpt;
        mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions    realTimeCorrOpt3D;
        mapping::scan_matching::proto::CeresScanMatcherOptions3D                ceresScan3DTrajOpt;
        common::proto::CeresSolverOptions                                       ceresSolverOpt3DTraj;
        mapping::scan_matching::proto::IntensityCostFunctionOptions*            intensityCostOpt;
        mapping::proto::MotionFilterOptions                                     motionFilter3dOpt;
        mapping::proto::PoseExtrapolatorOptions                                 poseExtrapolator3dOpt;
        mapping::proto::ConstantVelocityPoseExtrapolatorOptions                 constVel3dOpt;
        mapping::proto::ImuBasedPoseExtrapolatorOptions                         imuBasedPoseExtrap3dOpt;
        common::proto::CeresSolverOptions                                       solverOptImuBased3d;
        mapping::proto::SubmapsOptions3D                                        submapsOpt3d;
        mapping::proto::RangeDataInserterOptions3D                              rangeDataInserterOpt3d;

};