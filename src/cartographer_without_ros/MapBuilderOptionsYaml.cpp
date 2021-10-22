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

#include <cartographer_without_ros/MapBuilderOptionsYaml.h>

MapBuilderOptionsYaml::MapBuilderOptionsYaml(std::string _path){
    setMapBuilderOptions(_path);
}

void MapBuilderOptionsYaml::setPoseGraphOptions(YAML::Node _config){
    poseGraphOpt_.set_optimize_every_n_nodes(_config["optimize_every_n_nodes"].as<int>());
    poseGraphOpt_.set_global_sampling_ratio(_config["global_sampling_ratio"].as<float>());
    poseGraphOpt_.set_global_constraint_search_after_n_seconds(_config["global_constraint_search_after_n_seconds"].as<float>());
    poseGraphOpt_.set_matcher_translation_weight(_config["matcher_translation_weight"].as<float>());
    poseGraphOpt_.set_matcher_rotation_weight(_config["matcher_rotation_weight"].as<float>());
    poseGraphOpt_.set_max_num_final_iterations(_config["max_num_final_iterations"].as<int>());
    poseGraphOpt_.set_log_residual_histograms(_config["log_residual_histograms"].as<bool>());

    constraintOpt_.set_sampling_ratio(_config["constraint_builder"]["sampling_ratio"].as<float>());
    constraintOpt_.set_max_constraint_distance(_config["constraint_builder"]["max_constraint_distance"].as<float>());
    constraintOpt_.set_min_score(_config["constraint_builder"]["min_score"].as<float>());
    constraintOpt_.set_global_localization_min_score(_config["constraint_builder"]["global_localization_min_score"].as<float>());
    constraintOpt_.set_loop_closure_rotation_weight(_config["constraint_builder"]["loop_closure_rotation_weight"].as<float>());
    constraintOpt_.set_loop_closure_translation_weight(_config["constraint_builder"]["loop_closure_translation_weight"].as<float>());
    constraintOpt_.set_log_matches(_config["constraint_builder"]["log_matches"].as<bool>());

    fastCorrScan2DOpt_.set_angular_search_window(_config["constraint_builder"]["fast_correlative_scan_matcher"]["angular_search_window"].as<float>());
    fastCorrScan2DOpt_.set_branch_and_bound_depth(_config["constraint_builder"]["fast_correlative_scan_matcher"]["branch_and_bound_depth"].as<int>());
    fastCorrScan2DOpt_.set_linear_search_window(_config["constraint_builder"]["fast_correlative_scan_matcher"]["linear_search_window"].as<float>());
    constraintOpt_.set_allocated_fast_correlative_scan_matcher_options(&fastCorrScan2DOpt_);

    ceresScan2DOpt_.set_occupied_space_weight(_config["constraint_builder"]["ceres_scan_matcher"]["occupied_space_weight"].as<float>());
    ceresScan2DOpt_.set_translation_weight(_config["constraint_builder"]["ceres_scan_matcher"]["translation_weight"].as<float>());
    ceresScan2DOpt_.set_rotation_weight(_config["constraint_builder"]["ceres_scan_matcher"]["rotation_weight"].as<float>());

    ceresSolver2DOpt_.set_use_nonmonotonic_steps(_config["constraint_builder"]["ceres_scan_matcher"]["ceres_solver_options"]["use_nonmonotonic_steps"].as<bool>());
    ceresSolver2DOpt_.set_max_num_iterations(_config["constraint_builder"]["ceres_scan_matcher"]["ceres_solver_options"]["max_num_iterations"].as<int>());
    ceresSolver2DOpt_.set_num_threads(_config["constraint_builder"]["ceres_scan_matcher"]["ceres_solver_options"]["num_threads"].as<int>());
    ceresScan2DOpt_.set_allocated_ceres_solver_options(&ceresSolver2DOpt_);
    constraintOpt_.set_allocated_ceres_scan_matcher_options(&ceresScan2DOpt_);

    fastCorrScan3DOpt.set_angular_search_window(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["angular_search_window"].as<float>());
    fastCorrScan3DOpt.set_branch_and_bound_depth(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["branch_and_bound_depth"].as<int>());
    fastCorrScan3DOpt.set_linear_xy_search_window(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["linear_xy_search_window"].as<float>());
    fastCorrScan3DOpt.set_linear_z_search_window(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["linear_z_search_window"].as<float>());
    fastCorrScan3DOpt.set_full_resolution_depth(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["full_resolution_depth"].as<int>());
    fastCorrScan3DOpt.set_min_rotational_score(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["min_rotational_score"].as<float>());
    fastCorrScan3DOpt.set_min_low_resolution_score(_config["constraint_builder"]["fast_correlative_scan_matcher_3d"]["min_low_resolution_score"].as<float>());
    constraintOpt_.set_allocated_fast_correlative_scan_matcher_options_3d(&fastCorrScan3DOpt);

    ceresScan3DOpt_.add_occupied_space_weight(_config["constraint_builder"]["ceres_scan_matcher_3d"]["occupied_space_weight_0"].as<float>());
    ceresScan3DOpt_.add_occupied_space_weight(_config["constraint_builder"]["ceres_scan_matcher_3d"]["occupied_space_weight_1"].as<float>());
    ceresScan3DOpt_.set_translation_weight(_config["constraint_builder"]["ceres_scan_matcher_3d"]["translation_weight"].as<float>());
    ceresScan3DOpt_.set_rotation_weight(_config["constraint_builder"]["ceres_scan_matcher_3d"]["rotation_weight"].as<float>());
    ceresScan3DOpt_.set_only_optimize_yaw(_config["constraint_builder"]["ceres_scan_matcher_3d"]["only_optimize_yaw"].as<bool>());

    ceresSolver3DOpt_.set_use_nonmonotonic_steps(_config["constraint_builder"]["ceres_scan_matcher_3d"]["ceres_solver_options"]["use_nonmonotonic_steps"].as<bool>());
    ceresSolver3DOpt_.set_max_num_iterations(_config["constraint_builder"]["ceres_scan_matcher_3d"]["ceres_solver_options"]["max_num_iterations"].as<int>());
    ceresSolver3DOpt_.set_num_threads(_config["constraint_builder"]["ceres_scan_matcher_3d"]["ceres_solver_options"]["num_threads"].as<int>());
    ceresScan3DOpt_.set_allocated_ceres_solver_options(&ceresSolver3DOpt_);
    constraintOpt_.set_allocated_ceres_scan_matcher_options_3d(&ceresScan3DOpt_);
    poseGraphOpt_.set_allocated_constraint_builder_options(&constraintOpt_);

    optimProblemOpt_.set_fix_z_in_3d(_config["optimization_problem"]["fix_z_in_3d"].as<bool>());
    optimProblemOpt_.set_fixed_frame_pose_rotation_weight(_config["optimization_problem"]["fixed_frame_pose_rotation_weight"].as<float>());
    optimProblemOpt_.set_fixed_frame_pose_use_tolerant_loss(_config["optimization_problem"]["fixed_frame_pose_use_tolerant_loss"].as<bool>());
    optimProblemOpt_.set_fixed_frame_pose_tolerant_loss_param_a(_config["optimization_problem"]["fixed_frame_pose_tolerant_loss_param_a"].as<int>());
    optimProblemOpt_.set_fixed_frame_pose_tolerant_loss_param_b(_config["optimization_problem"]["fixed_frame_pose_tolerant_loss_param_b"].as<int>());
    optimProblemOpt_.set_fixed_frame_pose_translation_weight(_config["optimization_problem"]["fixed_frame_pose_translation_weight"].as<float>());
    optimProblemOpt_.set_huber_scale(_config["optimization_problem"]["huber_scale"].as<float>());
    optimProblemOpt_.set_local_slam_pose_rotation_weight(_config["optimization_problem"]["local_slam_pose_rotation_weight"].as<float>());
    optimProblemOpt_.set_local_slam_pose_translation_weight(_config["optimization_problem"]["local_slam_pose_translation_weight"].as<float>());
    optimProblemOpt_.set_log_solver_summary(_config["optimization_problem"]["log_solver_summary"].as<bool>());
    optimProblemOpt_.set_odometry_rotation_weight(_config["optimization_problem"]["odometry_rotation_weight"].as<float>());
    optimProblemOpt_.set_odometry_translation_weight(_config["optimization_problem"]["odometry_translation_weight"].as<float>());
    optimProblemOpt_.set_rotation_weight(_config["optimization_problem"]["rotation_weight"].as<float>());
    optimProblemOpt_.set_acceleration_weight(_config["optimization_problem"]["acceleration_weight"].as<float>());
    optimProblemOpt_.set_use_online_imu_extrinsics_in_3d(_config["optimization_problem"]["use_online_imu_extrinsics_in_3d"].as<bool>());

    ceresSolverOptimProblemOpt_.set_use_nonmonotonic_steps(_config["optimization_problem"]["ceres_solver_options"]["use_nonmonotonic_steps"].as<bool>());
    ceresSolverOptimProblemOpt_.set_max_num_iterations(_config["optimization_problem"]["ceres_solver_options"]["max_num_iterations"].as<int>());
    ceresSolverOptimProblemOpt_.set_num_threads(_config["optimization_problem"]["ceres_solver_options"]["num_threads"].as<int>());
    optimProblemOpt_.set_allocated_ceres_solver_options(&ceresSolverOptimProblemOpt_);
    poseGraphOpt_.set_allocated_optimization_problem_options(&optimProblemOpt_);
}

void MapBuilderOptionsYaml::setMapBuilderOptions(std::string _path){
    std::string fileNameMapBuilder = "/map_builder.yaml";
    YAML::Node configMap = YAML::LoadFile(_path + fileNameMapBuilder);

    mapBuilderOpt_.set_num_background_threads(configMap["num_background_threads"].as<int>());
    mapBuilderOpt_.set_use_trajectory_builder_3d(configMap["use_trajectory_builder_3d"].as<bool>());
    mapBuilderOpt_.set_use_trajectory_builder_2d(configMap["use_trajectory_builder_2d"].as<bool>());
    mapBuilderOpt_.set_collate_by_trajectory(configMap["collate_by_trajectory"].as<bool>());
    
    std::string fileNamePoseGraph = configMap["pose_graph"].as<std::string>();
    YAML::Node configPoseGraph = YAML::LoadFile(_path + fileNamePoseGraph);
    setPoseGraphOptions(configPoseGraph);
    mapBuilderOpt_.set_allocated_pose_graph_options(&poseGraphOpt_);
}

