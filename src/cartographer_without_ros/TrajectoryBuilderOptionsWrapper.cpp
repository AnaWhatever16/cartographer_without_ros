#include <cartographer_without_ros/TrajectoryBuilderOptionsWrapper.h>

TrajectoryBuilderOptionsWrapper::TrajectoryBuilderOptionsWrapper(YAML::Node _config){
    setTrajectoryBuilderOptions(_config);
}

void TrajectoryBuilderOptionsWrapper::setTrajectoryBuilder2DOptions(YAML::Node _config){
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

void TrajectoryBuilderOptionsWrapper::setTrajectoryBuilder3DOptions(YAML::Node _config){
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

void TrajectoryBuilderOptionsWrapper::setTrajectoryBuilderOptions(YAML::Node _config){
    setTrajectoryBuilder2DOptions(_config);
    trajBuilderOpt.set_allocated_trajectory_builder_2d_options(&traj2dOpt);

    setTrajectoryBuilder3DOptions(_config);
    trajBuilderOpt.set_allocated_trajectory_builder_3d_options(&traj3dOpt);
}
