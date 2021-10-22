#include <cartographer_without_ros/MapBuilderOptionsWrapper.h>

MapBuilderOptionsWrapper::MapBuilderOptionsWrapper(YAML::Node _config){
    setMapBuilderOptions(_config);
}

void MapBuilderOptionsWrapper::setPoseGraphOptions(YAML::Node _config){
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

void MapBuilderOptionsWrapper::setMapBuilderOptions(YAML::Node _config){
    mapBuilderOpt.set_num_background_threads(4);
    mapBuilderOpt.set_use_trajectory_builder_3d(false);
    mapBuilderOpt.set_use_trajectory_builder_2d(true);
    mapBuilderOpt.set_collate_by_trajectory(false);

    setPoseGraphOptions(_config);
    mapBuilderOpt.set_allocated_pose_graph_options(&poseGraphOpt);
}

