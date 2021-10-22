#include <cartographer_without_ros/TrajectoryBuilderOptionsYaml.h>

TrajectoryBuilderOptionsYaml::TrajectoryBuilderOptionsYaml(std::string _path){
    setTrajectoryBuilderOptions(_path);
}

void TrajectoryBuilderOptionsYaml::setTrajectoryBuilder2DOptions(YAML::Node _config){
    traj2dOpt_.set_use_imu_data(_config["use_imu_data"].as<bool>());
    traj2dOpt_.set_imu_gravity_time_constant(_config["imu_gravity_time_constant"].as<float>());
    traj2dOpt_.set_max_range(_config["max_range"].as<float>());
    traj2dOpt_.set_min_z(_config["min_z"].as<float>());
    traj2dOpt_.set_min_range(_config["min_range"].as<float>());
    traj2dOpt_.set_max_z(_config["max_z"].as<float>());
    traj2dOpt_.set_missing_data_ray_length(_config["missing_data_ray_length"].as<float>());
    traj2dOpt_.set_num_accumulated_range_data(_config["num_accumulated_range_data"].as<int>());
    traj2dOpt_.set_use_online_correlative_scan_matching(_config["use_online_correlative_scan_matching"].as<bool>());
    traj2dOpt_.set_voxel_filter_size(_config["voxel_filter_size"].as<float>());

    adaptVoxelOpt_.set_max_length(_config["adaptive_voxel_filter"]["max_length"].as<float>());
    adaptVoxelOpt_.set_max_range(_config["adaptive_voxel_filter"]["max_range"].as<float>());
    adaptVoxelOpt_.set_min_num_points(_config["adaptive_voxel_filter"]["min_num_points"].as<int>());
    traj2dOpt_.set_allocated_adaptive_voxel_filter_options(&adaptVoxelOpt_);

    loopClosureAdaptVoxelOpt_.set_max_length(_config["loop_closure_adaptive_voxel_filter"]["max_length"].as<float>());
    loopClosureAdaptVoxelOpt_.set_max_range(_config["loop_closure_adaptive_voxel_filter"]["max_range"].as<float>());
    loopClosureAdaptVoxelOpt_.set_min_num_points(_config["loop_closure_adaptive_voxel_filter"]["min_num_points"].as<int>());
    traj2dOpt_.set_allocated_loop_closure_adaptive_voxel_filter_options(&loopClosureAdaptVoxelOpt_);

    realTimeCorr2dOpt_.set_linear_search_window(_config["real_time_correlative_scan_matcher"]["linear_search_window"].as<float>());
    realTimeCorr2dOpt_.set_rotation_delta_cost_weight(_config["real_time_correlative_scan_matcher"]["rotation_delta_cost_weight"].as<float>());
    realTimeCorr2dOpt_.set_translation_delta_cost_weight(_config["real_time_correlative_scan_matcher"]["translation_delta_cost_weight"].as<float>());
    realTimeCorr2dOpt_.set_angular_search_window(_config["real_time_correlative_scan_matcher"]["angular_search_window"].as<float>());
    traj2dOpt_.set_allocated_real_time_correlative_scan_matcher_options(&realTimeCorr2dOpt_);

    ceresScan2dTrajOpt_.set_occupied_space_weight(_config["ceres_scan_matcher"]["occupied_space_weight"].as<float>());
    ceresScan2dTrajOpt_.set_rotation_weight(_config["ceres_scan_matcher"]["rotation_weight"].as<float>());
    ceresScan2dTrajOpt_.set_translation_weight(_config["ceres_scan_matcher"]["translation_weight"].as<float>());

    ceresSolver2dTrajOpt_.set_max_num_iterations(_config["ceres_scan_matcher"]["ceres_solver_options"]["max_num_iterations"].as<float>());
    ceresSolver2dTrajOpt_.set_num_threads(_config["ceres_scan_matcher"]["ceres_solver_options"]["num_threads"].as<float>());
    ceresSolver2dTrajOpt_.set_use_nonmonotonic_steps(_config["ceres_scan_matcher"]["ceres_solver_options"]["use_nonmonotonic_steps"].as<bool>());
    ceresScan2dTrajOpt_.set_allocated_ceres_solver_options(&ceresSolver2dTrajOpt_);
    traj2dOpt_.set_allocated_ceres_scan_matcher_options(&ceresScan2dTrajOpt_);

    motionFilter2dOpt_.set_max_angle_radians(_config["motion_filter"]["max_angle_radians"].as<float>());
    motionFilter2dOpt_.set_max_distance_meters(_config["motion_filter"]["max_distance_meters"].as<float>());
    motionFilter2dOpt_.set_max_time_seconds(_config["motion_filter"]["max_time_seconds"].as<float>());
    traj2dOpt_.set_allocated_motion_filter_options(&motionFilter2dOpt_);

    poseExtrapolator2dOpt_.set_use_imu_based(_config["pose_extrapolator"]["use_imu_based"].as<bool>());

    constVel2dOpt_.set_imu_gravity_time_constant(_config["pose_extrapolator"]["constant_velocity"]["imu_gravity_time_constant"].as<float>());
    constVel2dOpt_.set_pose_queue_duration(_config["pose_extrapolator"]["constant_velocity"]["pose_queue_duration"].as<float>());
    poseExtrapolator2dOpt_.set_allocated_constant_velocity(&constVel2dOpt_);

    imuBasedPoseExtrap2dOpt_.set_gravity_constant(_config["pose_extrapolator"]["imu_based"]["gravity_constant"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_imu_acceleration_weight(_config["pose_extrapolator"]["imu_based"]["imu_acceleration_weight"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_imu_rotation_weight(_config["pose_extrapolator"]["imu_based"]["imu_rotation_weight"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_odometry_rotation_weight(_config["pose_extrapolator"]["imu_based"]["odometry_rotation_weight"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_odometry_translation_weight(_config["pose_extrapolator"]["imu_based"]["odometry_translation_weight"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_pose_queue_duration(_config["pose_extrapolator"]["imu_based"]["pose_queue_duration"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_pose_rotation_weight(_config["pose_extrapolator"]["imu_based"]["pose_rotation_weight"].as<float>());
    imuBasedPoseExtrap2dOpt_.set_pose_translation_weight(_config["pose_extrapolator"]["imu_based"]["pose_translation_weight"].as<float>());

    solverImuBased2dOpt_.set_max_num_iterations(_config["pose_extrapolator"]["imu_based"]["solver_options"]["max_num_iterations"].as<int>());
    solverImuBased2dOpt_.set_num_threads(_config["pose_extrapolator"]["imu_based"]["solver_options"]["num_threads"].as<int>());
    solverImuBased2dOpt_.set_use_nonmonotonic_steps(_config["pose_extrapolator"]["imu_based"]["solver_options"]["use_nonmonotonic_steps"].as<bool>());
    imuBasedPoseExtrap2dOpt_.set_allocated_solver_options(&solverImuBased2dOpt_);
    poseExtrapolator2dOpt_.set_allocated_imu_based(&imuBasedPoseExtrap2dOpt_);
    traj2dOpt_.set_allocated_pose_extrapolator_options(&poseExtrapolator2dOpt_);

    submaps2dOpt_.set_num_range_data(_config["submaps"]["num_range_data"].as<int>());

    grid2dOpt_.set_grid_type(mapping::proto::GridOptions2D_GridType::GridOptions2D_GridType_PROBABILITY_GRID);
    grid2dOpt_.set_resolution(_config["submaps"]["grid_options_2d"]["resolution"].as<float>());
    submaps2dOpt_.set_allocated_grid_options_2d(&grid2dOpt_);

    rangeDataInserter2dOpt_.set_range_data_inserter_type(mapping::proto::RangeDataInserterOptions_RangeDataInserterType::RangeDataInserterOptions_RangeDataInserterType_PROBABILITY_GRID_INSERTER_2D);

    tsdfRangeInsert2dOpt_.set_truncation_distance(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["truncation_distance"].as<float>());
    tsdfRangeInsert2dOpt_.set_maximum_weight(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["maximum_weight"].as<float>());
    tsdfRangeInsert2dOpt_.set_project_sdf_distance_to_scan_normal(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["project_sdf_distance_to_scan_normal"].as<bool>());
    tsdfRangeInsert2dOpt_.set_update_free_space(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["update_free_space"].as<bool>());
    tsdfRangeInsert2dOpt_.set_update_weight_angle_scan_normal_to_ray_kernel_bandwidth(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["update_weight_angle_scan_normal_to_ray_kernel_bandwidth"].as<float>());
    tsdfRangeInsert2dOpt_.set_update_weight_distance_cell_to_hit_kernel_bandwidth(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["update_weight_distance_cell_to_hit_kernel_bandwidth"].as<float>());
    tsdfRangeInsert2dOpt_.set_update_weight_range_exponent(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["update_weight_range_exponent"].as<float>());

    normalEstimation2dOpt_.set_num_normal_samples(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["normal_estimation_options"]["num_normal_samples"].as<int>());
    normalEstimation2dOpt_.set_sample_radius(_config["submaps"]["range_data_inserter"]["tsdf_range_data_inserter"]["normal_estimation_options"]["sample_radius"].as<float>());
    tsdfRangeInsert2dOpt_.set_allocated_normal_estimation_options(&normalEstimation2dOpt_);
    rangeDataInserter2dOpt_.set_allocated_tsdf_range_data_inserter_options_2d(&tsdfRangeInsert2dOpt_);

    probGridRangeDataInsert2DOpt_.set_hit_probability(_config["submaps"]["range_data_inserter"]["probability_grid_range_data_inserter"]["hit_probability"].as<float>());
    probGridRangeDataInsert2DOpt_.set_insert_free_space(_config["submaps"]["range_data_inserter"]["probability_grid_range_data_inserter"]["insert_free_space"].as<bool>());
    probGridRangeDataInsert2DOpt_.set_miss_probability(_config["submaps"]["range_data_inserter"]["probability_grid_range_data_inserter"]["miss_probability"].as<float>());
    rangeDataInserter2dOpt_.set_allocated_probability_grid_range_data_inserter_options_2d(&probGridRangeDataInsert2DOpt_);
    submaps2dOpt_.set_allocated_range_data_inserter_options(&rangeDataInserter2dOpt_);
    traj2dOpt_.set_allocated_submaps_options(&submaps2dOpt_);
}

void TrajectoryBuilderOptionsYaml::setTrajectoryBuilder3DOptions(YAML::Node _config){
    traj3dOpt_.set_imu_gravity_time_constant(_config["imu_gravity_time_constant"].as<float>());
    traj3dOpt_.set_max_range(_config["max_range"].as<float>());
    traj3dOpt_.set_min_range(_config["min_range"].as<float>());
    traj3dOpt_.set_num_accumulated_range_data(_config["num_accumulated_range_data"].as<int>());
    traj3dOpt_.set_voxel_filter_size(_config["voxel_filter_size"].as<float>());
    traj3dOpt_.set_use_online_correlative_scan_matching(_config["use_online_correlative_scan_matching"].as<bool>());
    traj3dOpt_.set_rotational_histogram_size(_config["rotational_histogram_size"].as<int>());
    traj3dOpt_.set_use_intensities(_config["use_intensities"].as<bool>());

    highResAdaptVoxelOpt_.set_max_range(_config["high_resolution_adaptive_voxel_filter"]["max_range"].as<float>());
    highResAdaptVoxelOpt_.set_min_num_points(_config["high_resolution_adaptive_voxel_filter"]["min_num_points"].as<int>());
    highResAdaptVoxelOpt_.set_max_length(_config["high_resolution_adaptive_voxel_filter"]["max_length"].as<float>());
    traj3dOpt_.set_allocated_high_resolution_adaptive_voxel_filter_options(&highResAdaptVoxelOpt_);

    lowResAdaptVoxelOpt_.set_max_range(_config["low_resolution_adaptive_voxel_filter"]["max_range"].as<int>());
    lowResAdaptVoxelOpt_.set_min_num_points(_config["low_resolution_adaptive_voxel_filter"]["min_num_points"].as<int>());
    lowResAdaptVoxelOpt_.set_max_length(_config["low_resolution_adaptive_voxel_filter"]["max_length"].as<float>());
    traj3dOpt_.set_allocated_low_resolution_adaptive_voxel_filter_options(&lowResAdaptVoxelOpt_);

    realTimeCorr3dOpt_.set_linear_search_window(_config["real_time_correlative_scan_matcher"]["linear_search_window"].as<float>());
    realTimeCorr3dOpt_.set_rotation_delta_cost_weight(_config["real_time_correlative_scan_matcher"]["rotation_delta_cost_weight"].as<float>());
    realTimeCorr3dOpt_.set_translation_delta_cost_weight(_config["real_time_correlative_scan_matcher"]["translation_delta_cost_weight"].as<float>());
    realTimeCorr3dOpt_.set_angular_search_window(_config["real_time_correlative_scan_matcher"]["angular_search_window"].as<float>());
    traj3dOpt_.set_allocated_real_time_correlative_scan_matcher_options(&realTimeCorr3dOpt_);

    ceresScan3dTrajOpt_.add_occupied_space_weight(_config["ceres_scan_matcher"]["occupied_space_weight_0"].as<float>());
    ceresScan3dTrajOpt_.add_occupied_space_weight(_config["ceres_scan_matcher"]["occupied_space_weight_1"].as<float>());
    ceresScan3dTrajOpt_.set_rotation_weight(_config["ceres_scan_matcher"]["rotation_weight"].as<float>());
    ceresScan3dTrajOpt_.set_translation_weight(_config["ceres_scan_matcher"]["translation_weight"].as<float>());
    ceresScan3dTrajOpt_.set_only_optimize_yaw(_config["ceres_scan_matcher"]["only_optimize_yaw"].as<bool>());

    intensityCostOpt_ = ceresScan3dTrajOpt_.add_intensity_cost_function_options();
    intensityCostOpt_->set_weight(_config["ceres_scan_matcher"]["intensity_cost_function_options_0"]["weight"].as<float>());
    intensityCostOpt_->set_huber_scale(_config["ceres_scan_matcher"]["intensity_cost_function_options_0"]["huber_scale"].as<float>());
    intensityCostOpt_->set_intensity_threshold(_config["ceres_scan_matcher"]["intensity_cost_function_options_0"]["intensity_threshold"].as<float>());

    ceresSolver3dTrajOpt_.set_max_num_iterations(_config["ceres_scan_matcher"]["ceres_solver_options"]["max_num_iterations"].as<int>());
    ceresSolver3dTrajOpt_.set_num_threads(_config["ceres_scan_matcher"]["ceres_solver_options"]["num_threads"].as<int>());
    ceresSolver3dTrajOpt_.set_use_nonmonotonic_steps(_config["ceres_scan_matcher"]["ceres_solver_options"]["use_nonmonotonic_steps"].as<bool>());
    ceresScan3dTrajOpt_.set_allocated_ceres_solver_options(&ceresSolver3dTrajOpt_);
    traj3dOpt_.set_allocated_ceres_scan_matcher_options(&ceresScan3dTrajOpt_);

    motionFilter3dOpt_.set_max_time_seconds(_config["motion_filter"]["max_time_seconds"].as<float>());
    motionFilter3dOpt_.set_max_distance_meters(_config["motion_filter"]["max_distance_meters"].as<float>());
    motionFilter3dOpt_.set_max_angle_radians(_config["motion_filter"]["max_angle_radians"].as<float>());
    traj3dOpt_.set_allocated_motion_filter_options(&motionFilter3dOpt_);

    poseExtrapolator3dOpt_.set_use_imu_based(_config["pose_extrapolator"]["use_imu_based"].as<bool>());

    constVel3dOpt_.set_imu_gravity_time_constant(_config["pose_extrapolator"]["constant_velocity"]["imu_gravity_time_constant"].as<float>());
    constVel3dOpt_.set_pose_queue_duration(_config["pose_extrapolator"]["constant_velocity"]["pose_queue_duration"].as<float>());
    poseExtrapolator3dOpt_.set_allocated_constant_velocity(&constVel3dOpt_);

    imuBasedPoseExtrap3dOpt_.set_gravity_constant(_config["pose_extrapolator"]["imu_based"]["gravity_constant"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_imu_acceleration_weight(_config["pose_extrapolator"]["imu_based"]["imu_acceleration_weight"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_imu_rotation_weight(_config["pose_extrapolator"]["imu_based"]["imu_rotation_weight"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_odometry_rotation_weight(_config["pose_extrapolator"]["imu_based"]["odometry_rotation_weight"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_odometry_translation_weight(_config["pose_extrapolator"]["imu_based"]["odometry_translation_weight"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_pose_queue_duration(_config["pose_extrapolator"]["imu_based"]["pose_queue_duration"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_pose_rotation_weight(_config["pose_extrapolator"]["imu_based"]["pose_rotation_weight"].as<float>());
    imuBasedPoseExtrap3dOpt_.set_pose_translation_weight(_config["pose_extrapolator"]["imu_based"]["pose_translation_weight"].as<float>());

    solverImuBased3dOpt_.set_max_num_iterations(_config["pose_extrapolator"]["imu_based"]["solver_options"]["max_num_iterations"].as<int>());
    solverImuBased3dOpt_.set_num_threads(_config["pose_extrapolator"]["imu_based"]["solver_options"]["num_threads"].as<int>());
    solverImuBased3dOpt_.set_use_nonmonotonic_steps(_config["pose_extrapolator"]["imu_based"]["solver_options"]["use_nonmonotonic_steps"].as<bool>());
    imuBasedPoseExtrap3dOpt_.set_allocated_solver_options(&solverImuBased3dOpt_);
    poseExtrapolator3dOpt_.set_allocated_imu_based(&imuBasedPoseExtrap3dOpt_);
    traj3dOpt_.set_allocated_pose_extrapolator_options(&poseExtrapolator3dOpt_);

    submaps3dOpt_.set_num_range_data(_config["submaps"]["num_range_data"].as<int>());
    submaps3dOpt_.set_high_resolution(_config["submaps"]["high_resolution"].as<float>());
    submaps3dOpt_.set_high_resolution_max_range(_config["submaps"]["high_resolution_max_range"].as<float>());
    submaps3dOpt_.set_low_resolution(_config["submaps"]["low_resolution"].as<float>());
    submaps3dOpt_.set_num_range_data(_config["submaps"]["num_range_data"].as<float>());

    rangeDataInserter3dOpt_.set_hit_probability(_config["submaps"]["range_data_inserter"]["hit_probability"].as<float>());
    rangeDataInserter3dOpt_.set_intensity_threshold(_config["submaps"]["range_data_inserter"]["intensity_threshold"].as<int>());
    rangeDataInserter3dOpt_.set_miss_probability(_config["submaps"]["range_data_inserter"]["miss_probability"].as<float>());
    rangeDataInserter3dOpt_.set_num_free_space_voxels(_config["submaps"]["range_data_inserter"]["num_free_space_voxels"].as<int>());
    submaps3dOpt_.set_allocated_range_data_inserter_options(&rangeDataInserter3dOpt_);
    traj3dOpt_.set_allocated_submaps_options(&submaps3dOpt_);
}

void TrajectoryBuilderOptionsYaml::setTrajectoryBuilderOptions(std::string _path){
    std::string fileNameTrajBuilder = "/trajectory_builder.yaml";
    YAML::Node configTraj = YAML::LoadFile(_path + fileNameTrajBuilder);
    
    trajBuilderOpt_.set_collate_fixed_frame(configTraj["collate_fixed_frame"].as<bool>());
    trajBuilderOpt_.set_collate_landmarks(configTraj["collate_landmarks"].as<bool>());

    std::string fileNameTraj2D = configTraj["trajectory_builder_2d"].as<std::string>();
    YAML::Node configTraj2D = YAML::LoadFile(_path + fileNameTraj2D);
    setTrajectoryBuilder2DOptions(configTraj2D);
    trajBuilderOpt_.set_allocated_trajectory_builder_2d_options(&traj2dOpt_);
    
    std::string fileNameTraj3D = configTraj["trajectory_builder_3d"].as<std::string>();
    YAML::Node configTraj3D = YAML::LoadFile(_path + fileNameTraj3D);
    setTrajectoryBuilder3DOptions(configTraj3D);
    trajBuilderOpt_.set_allocated_trajectory_builder_3d_options(&traj3dOpt_);

}
