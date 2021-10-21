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
std::vector<transform::Rigid3f> gt;

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

void setPoseGraphOptions(){
    poseGraphOpt.set_optimize_every_n_nodes(90);
    poseGraphOpt.set_global_sampling_ratio(0.003);
    poseGraphOpt.set_global_constraint_search_after_n_seconds(10);
    poseGraphOpt.set_matcher_translation_weight(5e2);
    poseGraphOpt.set_matcher_rotation_weight(1.6e3);
    poseGraphOpt.set_max_num_final_iterations(200);
    poseGraphOpt.set_log_residual_histograms(true);

    constraintOpt.set_sampling_ratio(0.3);
    constraintOpt.set_max_constraint_distance(15);
    constraintOpt.set_min_score(0.55);
    constraintOpt.set_global_localization_min_score(0.6);
    constraintOpt.set_loop_closure_rotation_weight(1e5);
    constraintOpt.set_loop_closure_translation_weight(1.1e4);
    constraintOpt.set_log_matches(true);

    fastCorrScan2DOpt.set_angular_search_window(M_PI/6);
    fastCorrScan2DOpt.set_branch_and_bound_depth(7);
    fastCorrScan2DOpt.set_linear_search_window(7);
    constraintOpt.set_allocated_fast_correlative_scan_matcher_options(&fastCorrScan2DOpt);

    ceresScan2DPoseGraphOpt.set_occupied_space_weight(20);
    ceresScan2DPoseGraphOpt.set_translation_weight(10);
    ceresScan2DPoseGraphOpt.set_rotation_weight(1);

    ceresSolverOpt2DPoseGraph.set_use_nonmonotonic_steps(true);
    ceresSolverOpt2DPoseGraph.set_max_num_iterations(10);
    ceresSolverOpt2DPoseGraph.set_num_threads(1);
    ceresScan2DPoseGraphOpt.set_allocated_ceres_solver_options(&ceresSolverOpt2DPoseGraph);
    constraintOpt.set_allocated_ceres_scan_matcher_options(&ceresScan2DPoseGraphOpt);

    fastCorrScan3DOpt.set_angular_search_window(M_PI/12);
    fastCorrScan3DOpt.set_branch_and_bound_depth(8);
    fastCorrScan3DOpt.set_linear_xy_search_window(5);
    fastCorrScan3DOpt.set_linear_z_search_window(1);
    fastCorrScan3DOpt.set_full_resolution_depth(3);
    fastCorrScan3DOpt.set_min_rotational_score(0.77);
    fastCorrScan3DOpt.set_min_low_resolution_score(0.55);
    constraintOpt.set_allocated_fast_correlative_scan_matcher_options_3d(&fastCorrScan3DOpt);

    ceresScan3DPoseGraphOpt.add_occupied_space_weight(5);
    ceresScan3DPoseGraphOpt.add_occupied_space_weight(30);
    ceresScan3DPoseGraphOpt.set_translation_weight(10);
    ceresScan3DPoseGraphOpt.set_rotation_weight(1);
    ceresScan3DPoseGraphOpt.set_only_optimize_yaw(false);

    ceresSolverOpt3DPoseGraph.set_use_nonmonotonic_steps(false);
    ceresSolverOpt3DPoseGraph.set_max_num_iterations(10);
    ceresSolverOpt3DPoseGraph.set_num_threads(1);
    ceresScan3DPoseGraphOpt.set_allocated_ceres_solver_options(&ceresSolverOpt3DPoseGraph);
    constraintOpt.set_allocated_ceres_scan_matcher_options_3d(&ceresScan3DPoseGraphOpt);
    poseGraphOpt.set_allocated_constraint_builder_options(&constraintOpt);

    optimizationProblemOpt.set_fix_z_in_3d(false);
    optimizationProblemOpt.set_fixed_frame_pose_rotation_weight(1e2);
    optimizationProblemOpt.set_fixed_frame_pose_use_tolerant_loss(false);
    optimizationProblemOpt.set_fixed_frame_pose_tolerant_loss_param_a(1);
    optimizationProblemOpt.set_fixed_frame_pose_tolerant_loss_param_b(1);
    optimizationProblemOpt.set_fixed_frame_pose_translation_weight(1e1);
    optimizationProblemOpt.set_huber_scale(1e1);
    optimizationProblemOpt.set_local_slam_pose_rotation_weight(1e5);
    optimizationProblemOpt.set_local_slam_pose_translation_weight(1e5);
    optimizationProblemOpt.set_log_solver_summary(false);
    optimizationProblemOpt.set_odometry_rotation_weight(1e5);
    optimizationProblemOpt.set_odometry_translation_weight(1e5);
    optimizationProblemOpt.set_rotation_weight(1.6e4);
    optimizationProblemOpt.set_acceleration_weight(1.1e2);
    optimizationProblemOpt.set_use_online_imu_extrinsics_in_3d(true);

    ceresSolverOptOptimizationProblem.set_use_nonmonotonic_steps(false);
    ceresSolverOptOptimizationProblem.set_max_num_iterations(50);
    ceresSolverOptOptimizationProblem.set_num_threads(7);
    optimizationProblemOpt.set_allocated_ceres_solver_options(&ceresSolverOptOptimizationProblem);
    poseGraphOpt.set_allocated_optimization_problem_options(&optimizationProblemOpt);
}

void setMapBuilderOptions(){
    mapBuilderOpt.set_num_background_threads(4);
    mapBuilderOpt.set_use_trajectory_builder_3d(false);
    mapBuilderOpt.set_use_trajectory_builder_2d(true);
    mapBuilderOpt.set_collate_by_trajectory(false);

    setPoseGraphOptions();
    mapBuilderOpt.set_allocated_pose_graph_options(&poseGraphOpt);
}

void setTrajectoryBuilder2DOptions(){
    trajBuilderOpt.set_collate_fixed_frame(true);
    trajBuilderOpt.set_collate_landmarks(false);

    traj2dOpt.set_use_imu_data(false);
    traj2dOpt.set_imu_gravity_time_constant(10);
    traj2dOpt.set_max_range(30);
    traj2dOpt.set_min_z(-0.8);
    traj2dOpt.set_min_range(0);
    traj2dOpt.set_max_z(2);
    traj2dOpt.set_missing_data_ray_length(5);
    traj2dOpt.set_num_accumulated_range_data(1);
    traj2dOpt.set_use_online_correlative_scan_matching(false);
    traj2dOpt.set_voxel_filter_size(0.025);

    adaptVoxelOpt.set_max_length(0.5);
    adaptVoxelOpt.set_max_range(50);
    adaptVoxelOpt.set_min_num_points(200);
    traj2dOpt.set_allocated_adaptive_voxel_filter_options(&adaptVoxelOpt);

    loopClosureAdaptVoxelOpt.set_max_length(0.9);
    loopClosureAdaptVoxelOpt.set_max_range(50);
    loopClosureAdaptVoxelOpt.set_min_num_points(100);
    traj2dOpt.set_allocated_loop_closure_adaptive_voxel_filter_options(&loopClosureAdaptVoxelOpt);

    realTimeCorrOpt2D.set_linear_search_window(0.1);
    realTimeCorrOpt2D.set_rotation_delta_cost_weight(1e-1);
    realTimeCorrOpt2D.set_translation_delta_cost_weight(1e-1);
    realTimeCorrOpt2D.set_angular_search_window(M_PI/9);
    traj2dOpt.set_allocated_real_time_correlative_scan_matcher_options(&realTimeCorrOpt2D);

    ceresScan2DTrajOpt.set_occupied_space_weight(1);
    ceresScan2DTrajOpt.set_rotation_weight(40);
    ceresScan2DTrajOpt.set_translation_weight(10);

    ceresSolverOpt2DTraj.set_max_num_iterations(20);
    ceresSolverOpt2DTraj.set_num_threads(1);
    ceresSolverOpt2DTraj.set_use_nonmonotonic_steps(false);
    ceresScan2DTrajOpt.set_allocated_ceres_solver_options(&ceresSolverOpt2DTraj);
    traj2dOpt.set_allocated_ceres_scan_matcher_options(&ceresScan2DTrajOpt);

    motionFilter2dOpt.set_max_angle_radians(M_PI/180);
    motionFilter2dOpt.set_max_distance_meters(0.2);
    motionFilter2dOpt.set_max_time_seconds(5);
    traj2dOpt.set_allocated_motion_filter_options(&motionFilter2dOpt);

    poseExtrapolator2dOpt.set_use_imu_based(false);

    constVel2dOpt.set_imu_gravity_time_constant(10);
    constVel2dOpt.set_pose_queue_duration(0.001);
    poseExtrapolator2dOpt.set_allocated_constant_velocity(&constVel2dOpt);

    imuBasedPoseExtrap2dOpt.set_gravity_constant(9.806);
    imuBasedPoseExtrap2dOpt.set_imu_acceleration_weight(1);
    imuBasedPoseExtrap2dOpt.set_imu_rotation_weight(1);
    imuBasedPoseExtrap2dOpt.set_odometry_rotation_weight(1);
    imuBasedPoseExtrap2dOpt.set_odometry_translation_weight(1);
    imuBasedPoseExtrap2dOpt.set_pose_queue_duration(5);
    imuBasedPoseExtrap2dOpt.set_pose_rotation_weight(1);
    imuBasedPoseExtrap2dOpt.set_pose_translation_weight(1);

    solverOptImuBased2d.set_max_num_iterations(10);
    solverOptImuBased2d.set_num_threads(1);
    solverOptImuBased2d.set_use_nonmonotonic_steps(false);
    imuBasedPoseExtrap2dOpt.set_allocated_solver_options(&solverOptImuBased2d);
    poseExtrapolator2dOpt.set_allocated_imu_based(&imuBasedPoseExtrap2dOpt);
    traj2dOpt.set_allocated_pose_extrapolator_options(&poseExtrapolator2dOpt);

    submapsOpt2d.set_num_range_data(90);

    gridOpt2d.set_grid_type(mapping::proto::GridOptions2D_GridType::GridOptions2D_GridType_PROBABILITY_GRID);
    gridOpt2d.set_resolution(0.05);
    submapsOpt2d.set_allocated_grid_options_2d(&gridOpt2d);

    rangeDataInserterOpt2d.set_range_data_inserter_type(mapping::proto::RangeDataInserterOptions_RangeDataInserterType::RangeDataInserterOptions_RangeDataInserterType_PROBABILITY_GRID_INSERTER_2D);

    tsdfRangeInsertOpt2D.set_truncation_distance(0.3);
    tsdfRangeInsertOpt2D.set_maximum_weight(10);
    tsdfRangeInsertOpt2D.set_project_sdf_distance_to_scan_normal(true);
    tsdfRangeInsertOpt2D.set_update_free_space(false);
    tsdfRangeInsertOpt2D.set_update_weight_angle_scan_normal_to_ray_kernel_bandwidth(0.5);
    tsdfRangeInsertOpt2D.set_update_weight_distance_cell_to_hit_kernel_bandwidth(0.5);
    tsdfRangeInsertOpt2D.set_update_weight_range_exponent(0);

    normalEstimation2DOpt.set_num_normal_samples(4);
    normalEstimation2DOpt.set_sample_radius(0.5);
    tsdfRangeInsertOpt2D.set_allocated_normal_estimation_options(&normalEstimation2DOpt);
    rangeDataInserterOpt2d.set_allocated_tsdf_range_data_inserter_options_2d(&tsdfRangeInsertOpt2D);

    probGridRangeDataInsert2DOpt.set_hit_probability(0.55);
    probGridRangeDataInsert2DOpt.set_insert_free_space(true);
    probGridRangeDataInsert2DOpt.set_miss_probability(0.49);
    rangeDataInserterOpt2d.set_allocated_probability_grid_range_data_inserter_options_2d(&probGridRangeDataInsert2DOpt);
    submapsOpt2d.set_allocated_range_data_inserter_options(&rangeDataInserterOpt2d);
    traj2dOpt.set_allocated_submaps_options(&submapsOpt2d);
}

void setTrajectoryBuilder3DOptions(){
    traj3dOpt.set_imu_gravity_time_constant(10);
    traj3dOpt.set_max_range(60);
    traj3dOpt.set_min_range(1);
    traj3dOpt.set_num_accumulated_range_data(1);
    traj3dOpt.set_voxel_filter_size(0.15);
    traj3dOpt.set_use_online_correlative_scan_matching(false);
    traj3dOpt.set_rotational_histogram_size(120);
    traj3dOpt.set_use_intensities(false);

    highResAdaptVoxelOpt.set_max_range(15);
    highResAdaptVoxelOpt.set_min_num_points(150);
    highResAdaptVoxelOpt.set_max_length(2);
    traj3dOpt.set_allocated_high_resolution_adaptive_voxel_filter_options(&highResAdaptVoxelOpt);

    lowResAdaptVoxelOpt.set_max_range(60);
    lowResAdaptVoxelOpt.set_min_num_points(200);
    lowResAdaptVoxelOpt.set_max_length(4);
    traj3dOpt.set_allocated_low_resolution_adaptive_voxel_filter_options(&lowResAdaptVoxelOpt);

    realTimeCorrOpt3D.set_linear_search_window(0.15);
    realTimeCorrOpt3D.set_rotation_delta_cost_weight(1e-1);
    realTimeCorrOpt3D.set_translation_delta_cost_weight(1e-1);
    realTimeCorrOpt3D.set_angular_search_window(M_PI/180);
    traj3dOpt.set_allocated_real_time_correlative_scan_matcher_options(&realTimeCorrOpt3D);

    ceresScan3DTrajOpt.add_occupied_space_weight(1);
    ceresScan3DTrajOpt.add_occupied_space_weight(6);
    ceresScan3DTrajOpt.set_rotation_weight(4e2);
    ceresScan3DTrajOpt.set_translation_weight(5);
    ceresScan3DTrajOpt.set_only_optimize_yaw(false);

    intensityCostOpt = ceresScan3DTrajOpt.add_intensity_cost_function_options();
    intensityCostOpt->set_weight(0.5);
    intensityCostOpt->set_huber_scale(0.3);
    intensityCostOpt->set_intensity_threshold(40);

    ceresSolverOpt3DTraj.set_max_num_iterations(12);
    ceresSolverOpt3DTraj.set_num_threads(1);
    ceresSolverOpt3DTraj.set_use_nonmonotonic_steps(false);
    ceresScan3DTrajOpt.set_allocated_ceres_solver_options(&ceresSolverOpt3DTraj);
    traj3dOpt.set_allocated_ceres_scan_matcher_options(&ceresScan3DTrajOpt);

    motionFilter3dOpt.set_max_time_seconds(0.5);
    motionFilter3dOpt.set_max_distance_meters(0.1);
    motionFilter3dOpt.set_max_angle_radians(0.004);
    traj3dOpt.set_allocated_motion_filter_options(&motionFilter3dOpt);

    poseExtrapolator3dOpt.set_use_imu_based(false);

    constVel3dOpt.set_imu_gravity_time_constant(10);
    constVel3dOpt.set_pose_queue_duration(0.001);

    poseExtrapolator3dOpt.set_allocated_constant_velocity(&constVel3dOpt);

    imuBasedPoseExtrap3dOpt.set_gravity_constant(9.806);
    imuBasedPoseExtrap3dOpt.set_imu_acceleration_weight(1);
    imuBasedPoseExtrap3dOpt.set_imu_rotation_weight(1);
    imuBasedPoseExtrap3dOpt.set_odometry_rotation_weight(1);
    imuBasedPoseExtrap3dOpt.set_odometry_translation_weight(1);
    imuBasedPoseExtrap3dOpt.set_pose_queue_duration(5);
    imuBasedPoseExtrap3dOpt.set_pose_rotation_weight(1);
    imuBasedPoseExtrap3dOpt.set_pose_translation_weight(1);

    solverOptImuBased3d.set_max_num_iterations(10);
    solverOptImuBased3d.set_num_threads(1);
    solverOptImuBased3d.set_use_nonmonotonic_steps(false);
    imuBasedPoseExtrap3dOpt.set_allocated_solver_options(&solverOptImuBased3d);
    poseExtrapolator3dOpt.set_allocated_imu_based(&imuBasedPoseExtrap3dOpt);
    traj3dOpt.set_allocated_pose_extrapolator_options(&poseExtrapolator3dOpt);

    submapsOpt3d.set_num_range_data(4);
    submapsOpt3d.set_high_resolution(0.1);
    submapsOpt3d.set_high_resolution_max_range(20);
    submapsOpt3d.set_low_resolution(0.45);
    submapsOpt3d.set_num_range_data(160);

    rangeDataInserterOpt3d.set_hit_probability(0.55);
    rangeDataInserterOpt3d.set_intensity_threshold(40);
    rangeDataInserterOpt3d.set_miss_probability(0.49);
    rangeDataInserterOpt3d.set_num_free_space_voxels(2);
    submapsOpt3d.set_allocated_range_data_inserter_options(&rangeDataInserterOpt3d);
    traj3dOpt.set_allocated_submaps_options(&submapsOpt3d);
}

void setTrajectoryBuilderOptions(){
    setTrajectoryBuilder2DOptions();
    trajBuilderOpt.set_allocated_trajectory_builder_2d_options(&traj2dOpt);

    setTrajectoryBuilder3DOptions();
    trajBuilderOpt.set_allocated_trajectory_builder_3d_options(&traj3dOpt);
}

std::vector<sensor::TimedPointCloudData> GenerateFakeRangeMeasurements(double travel_distance, double duration, double time_step)
{
    const Eigen::Vector3f kDirection = Eigen::Vector3f(2., 1., 0.).normalized();
    std::vector<sensor::TimedPointCloudData> measurements;
    sensor::TimedPointCloud point_cloud;
    for (double angle = 0.; angle < M_PI; angle += 0.01){
        for (double height : {-0.4, -0.2, 0.0, 0.2, 0.4}){
            constexpr double kRadius = 5;
            point_cloud.push_back({Eigen::Vector3d{kRadius * std::cos(angle), kRadius * std::sin(angle), height}.cast<float>(), 0.});
        }
    }
    const Eigen::Vector3f kVelocity = kDirection * travel_distance / duration;
    auto local_to_global = transform::Rigid3f::Identity();
    for (double elapsed_time = 0.; elapsed_time < duration; elapsed_time += time_step){

        common::Time time = common::FromUniversal(123) + common::FromSeconds(elapsed_time);
        transform::Rigid3f global_pose = local_to_global * transform::Rigid3f::Translation(elapsed_time * kVelocity);
        gt.push_back(global_pose);
        sensor::TimedPointCloud ranges = sensor::TransformTimedPointCloud(point_cloud, global_pose.inverse());

        measurements.emplace_back(sensor::TimedPointCloudData{time, Eigen::Vector3f::Zero(), ranges});
    }
    return measurements;
}

mapping::MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback(){
    return [=](const int trajectory_id, const common::Time time, const transform::Rigid3d local_pose,
               sensor::RangeData range_data_in_local, const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult>){
        std::cout << local_pose << std::endl;
        local_slam_result_poses_.push_back(local_pose);
    };
}

int main(int _argc, char** _argv){
    setMapBuilderOptions();
    setTrajectoryBuilderOptions();

    auto mapBuilder = mapping::CreateMapBuilder(mapBuilderOpt);

    const mapping::TrajectoryBuilderInterface::SensorId kRangeSensorId{mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, "range"};
    int trajId = mapBuilder->AddTrajectoryBuilder({kRangeSensorId}, trajBuilderOpt, GetLocalSlamResultCallback());
    mapping::TrajectoryBuilderInterface* trajBuilder = mapBuilder->GetTrajectoryBuilder(trajId);

    constexpr double kDuration = 4.;        // Seconds.
    constexpr double kTimeStep = 0.1;       // Seconds.
    constexpr double kTravelDistance = 1.2; // Meters.

    const auto measurements = GenerateFakeRangeMeasurements(kTravelDistance, kDuration, kTimeStep);
    for (const auto& measurement : measurements) {
        trajBuilder->AddSensorData(kRangeSensorId.id, measurement);
    }

    for(unsigned i = 0; i < gt.size(); i++){
        std::cout << "-----------------------------------" << std::endl;
        std::cout  << local_slam_result_poses_[i]<<std::endl;
        std::cout << gt[i] << std::endl;
    }

    mapBuilder->FinishTrajectory(trajId);
    mapBuilder->pose_graph()->RunFinalOptimization();
    // tb.addOdometry....
    
    return 1;
}