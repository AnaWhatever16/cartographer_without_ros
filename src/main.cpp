#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <cartographer/common/configuration_file_resolver.h>

#if defined(WIN32)
    #include <filesystem>
    namespace fs = std::filesystem;
#else
    #include <experimental/filesystem>  // Not implemented until g++8
    namespace fs = std::experimental::filesystem;
#endif

using namespace cartographer;
std::vector<transform::Rigid3d> local_slam_result_poses_;
mapping::proto::PoseGraphOptions poseGraphOptions;
mapping::proto::LocalTrajectoryBuilderOptions2D traj2dOpt;
mapping::proto::LocalTrajectoryBuilderOptions3D traj3dOpt;
mapping::proto::MapBuilderOptions mapBuilderOptions;
mapping::proto::TrajectoryBuilderOptions trajBuilderOptions;
mapping::proto::SubmapsOptions2D submaps_options2d;
mapping::proto::SubmapsOptions3D submaps_options3d;
mapping::proto::RangeDataInserterOptions rangeDataInserterOptions2d;
mapping::proto::RangeDataInserterOptions3D rangeDataInserterOptions3d;
mapping::proto::PoseExtrapolatorOptions poseExtrapolatorOpt;
mapping::proto::ConstantVelocityPoseExtrapolatorOptions constVelOpt;

void setMapBuilderOptions(){
    mapBuilderOptions.set_num_background_threads(4);
    mapBuilderOptions.set_use_trajectory_builder_3d(false);
    mapBuilderOptions.set_use_trajectory_builder_2d(true);
    poseGraphOptions.set_optimize_every_n_nodes(0);
    poseGraphOptions.set_global_sampling_ratio(0.05);
    poseGraphOptions.set_global_constraint_search_after_n_seconds(0);
    mapBuilderOptions.release_pose_graph_options();
    mapBuilderOptions.set_allocated_pose_graph_options(&poseGraphOptions);
}

void setTrajectoryBuilderOptions(){
    submaps_options2d.set_num_range_data(4);
    rangeDataInserterOptions2d.set_range_data_inserter_type(mapping::proto::RangeDataInserterOptions_RangeDataInserterType::RangeDataInserterOptions_RangeDataInserterType_PROBABILITY_GRID_INSERTER_2D);
    submaps_options2d.set_allocated_range_data_inserter_options(&rangeDataInserterOptions2d);
    traj2dOpt.set_allocated_submaps_options(&submaps_options2d);
    traj2dOpt.set_use_imu_data(false);
    poseExtrapolatorOpt.set_use_imu_based(false);
    constVelOpt.set_imu_gravity_time_constant(10);
    //constVelOpt.set_pose_queue_duration(0.001);
    poseExtrapolatorOpt.set_allocated_constant_velocity(&constVelOpt);
    traj2dOpt.set_allocated_pose_extrapolator_options(&poseExtrapolatorOpt);
    trajBuilderOptions.set_allocated_trajectory_builder_2d_options(&traj2dOpt);
    
    // submaps_options3d.set_num_range_data(4);
    // submaps_options3d.set_allocated_range_data_inserter_options(&rangeDataInserterOptions3d);
    // traj3dOpt.set_allocated_submaps_options(&submaps_options3d);   
    // trajBuilderOptions.set_allocated_trajectory_builder_3d_options(&traj3dOpt);
}

std::vector<sensor::TimedPointCloudData>
GenerateFakeRangeMeasurements(double travel_distance, double duration,
                              double time_step) {
    const Eigen::Vector3f kDirection = Eigen::Vector3f(2., 1., 0.).normalized();
    std::vector<sensor::TimedPointCloudData> measurements;
    sensor::TimedPointCloud point_cloud;
    for (double angle = 0.; angle < M_PI; angle += 0.01) {
        for (double height : {-0.4, -0.2, 0.0, 0.2, 0.4}) {
        constexpr double kRadius = 5;
        point_cloud.push_back({Eigen::Vector3d{kRadius * std::cos(angle),
                                                kRadius * std::sin(angle), height}
                                    .cast<float>(),
                                0.});
        }
    }
    const Eigen::Vector3f kVelocity = kDirection * travel_distance / duration;
    auto local_to_global = transform::Rigid3f::Identity();
    for (double elapsed_time = 0.; elapsed_time < duration;
        elapsed_time += time_step) {
        common::Time time =
            common::FromUniversal(123) +
            common::FromSeconds(elapsed_time);
        transform::Rigid3f global_pose =
            local_to_global *
            transform::Rigid3f::Translation(elapsed_time * kVelocity);
        sensor::TimedPointCloud ranges =
            sensor::TransformTimedPointCloud(point_cloud,
                                                        global_pose.inverse());
        measurements.emplace_back(sensor::TimedPointCloudData{
            time, Eigen::Vector3f::Zero(), ranges});
    }
    return measurements;
}

mapping::MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const common::Time time,
               const transform::Rigid3d local_pose,
               sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
        std::cout << local_pose << std::endl;
        local_slam_result_poses_.push_back(local_pose);
    };
}

int main(int _argc, char** _argv){
    setMapBuilderOptions();
    setTrajectoryBuilderOptions();
    auto mapBuilder = mapping::CreateMapBuilder(mapBuilderOptions);

    const mapping::TrajectoryBuilderInterface::SensorId kRangeSensorId{mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, "range"};
    int trajId = mapBuilder->AddTrajectoryBuilder({kRangeSensorId}, trajBuilderOptions, GetLocalSlamResultCallback());
    mapping::TrajectoryBuilderInterface* trajBuilder = mapBuilder->GetTrajectoryBuilder(trajId);

    constexpr double kDuration = 4.;         // Seconds.
    constexpr double kTimeStep = 0.1;        // Seconds.
    constexpr double kTravelDistance = 1.2;  // Meters.

    const auto measurements = GenerateFakeRangeMeasurements(kTravelDistance, kDuration, kTimeStep);
    for (const auto& measurement : measurements) {
        trajBuilder->AddSensorData(kRangeSensorId.id, measurement);
    }

    mapBuilder->FinishTrajectory(trajId);
    mapBuilder->pose_graph()->RunFinalOptimization();
    // tb.addOdometry....

    return 1;
}