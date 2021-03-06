//------------------------------------------------------------------------------------------------------------------------------
// Copyright (c) 2021 Ana Maria Casado Fauli (anacasadofauli@gmail.com)
//------------------------------------------------------------------------------------------------------------------------------
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
// documentation files (the "Software"), to deal in the Software without restriction, including without 
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
// Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------------------------------------------------------

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;

/// This class is in charge of setting all the parameters of the TrajectoryBuilder, given the path where all the config files should exist.
class TrajectoryBuilderOptionsYaml{
    public:
        /// Constructor.
        /// \param _path path to folder of config files
        TrajectoryBuilderOptionsYaml(std::string _path);

        /// This method will return the configuration for the TrajectoryBuilder that we desire.
        mapping::proto::TrajectoryBuilderOptions getTrajectoryOpt() { return trajBuilderOpt_; }

    private:
        void setTrajectoryBuilderOptions(std::string _path);
        void setTrajectoryBuilder2DOptions(YAML::Node _config);
        void setTrajectoryBuilder3DOptions(YAML::Node _config);

    private:
        mapping::proto::TrajectoryBuilderOptions                                trajBuilderOpt_;

        mapping::proto::LocalTrajectoryBuilderOptions2D                         traj2dOpt_;
        sensor::proto::AdaptiveVoxelFilterOptions                               adaptVoxelOpt_;
        sensor::proto::AdaptiveVoxelFilterOptions                               loopClosureAdaptVoxelOpt_;
        mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions    realTimeCorr2dOpt_;
        mapping::scan_matching::proto::CeresScanMatcherOptions2D                ceresScan2dTrajOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver2dTrajOpt_;
        mapping::proto::MotionFilterOptions                                     motionFilter2dOpt_;
        mapping::proto::PoseExtrapolatorOptions                                 poseExtrapolator2dOpt_;
        mapping::proto::ConstantVelocityPoseExtrapolatorOptions                 constVel2dOpt_;
        mapping::proto::ImuBasedPoseExtrapolatorOptions                         imuBasedPoseExtrap2dOpt_;
        common::proto::CeresSolverOptions                                       solverImuBased2dOpt_;
        mapping::proto::SubmapsOptions2D                                        submaps2dOpt_;
        mapping::proto::GridOptions2D                                           grid2dOpt_;
        mapping::proto::RangeDataInserterOptions                                rangeDataInserter2dOpt_;
        mapping::proto::ProbabilityGridRangeDataInserterOptions2D               probGridRangeDataInsert2DOpt_;
        mapping::proto::TSDFRangeDataInserterOptions2D                          tsdfRangeInsert2dOpt_;
        mapping::proto::NormalEstimationOptions2D                               normalEstimation2dOpt_;

        mapping::proto::LocalTrajectoryBuilderOptions3D                         traj3dOpt_;
        sensor::proto::AdaptiveVoxelFilterOptions                               highResAdaptVoxelOpt_;
        sensor::proto::AdaptiveVoxelFilterOptions                               lowResAdaptVoxelOpt_;
        mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions    realTimeCorr3dOpt_;
        mapping::scan_matching::proto::CeresScanMatcherOptions3D                ceresScan3dTrajOpt_;
        common::proto::CeresSolverOptions                                       ceresSolver3dTrajOpt_;
        mapping::scan_matching::proto::IntensityCostFunctionOptions*            intensityCostOpt_;
        mapping::proto::MotionFilterOptions                                     motionFilter3dOpt_;
        mapping::proto::PoseExtrapolatorOptions                                 poseExtrapolator3dOpt_;
        mapping::proto::ConstantVelocityPoseExtrapolatorOptions                 constVel3dOpt_;
        mapping::proto::ImuBasedPoseExtrapolatorOptions                         imuBasedPoseExtrap3dOpt_;
        common::proto::CeresSolverOptions                                       solverImuBased3dOpt_;
        mapping::proto::SubmapsOptions3D                                        submaps3dOpt_;
        mapping::proto::RangeDataInserterOptions3D                              rangeDataInserter3dOpt_;

};