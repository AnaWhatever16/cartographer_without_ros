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

// This script is based on one of the tests given by the original repository (map_builder_test.cc)
#include <cartographer_without_ros/MapBuilderOptionsYaml.h>
#include <cartographer_without_ros/TrajectoryBuilderOptionsYaml.h>
#include <iostream>
#include <vector>

std::vector<transform::Rigid3d> localSlamResultPoses;
std::vector<transform::Rigid3f> gt;

// This method generates fake point clouds, given the input params.
std::vector<sensor::TimedPointCloudData> GenerateFakeRangeMeasurements(double travel_distance, double duration, double time_step){
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
        gt.push_back(global_pose); // Our fake ground truth :)
        sensor::TimedPointCloud ranges = sensor::TransformTimedPointCloud(point_cloud, global_pose.inverse());

        measurements.emplace_back(sensor::TimedPointCloudData{time, Eigen::Vector3f::Zero(), ranges});
    }
    return measurements;
}

// Callback for the local pose results
mapping::MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback(){
    return [=](const int trajectory_id, const common::Time time, const transform::Rigid3d local_pose,
               sensor::RangeData range_data_in_local, const std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult>){
        std::cout << local_pose << std::endl;
        localSlamResultPoses.push_back(local_pose);
    };
}

// As an input, give the path to the config_files folder
int main(int _argc, char** _argv){
    // Parse configuration
    std::string path = _argv[1];
    MapBuilderOptionsYaml mapBuilderOpt(path);
    TrajectoryBuilderOptionsYaml trajBuilderOpt(path);

    // Create MapBuilder with configuration (global SLAM)
    auto mapBuilder = mapping::CreateMapBuilder(mapBuilderOpt.getMapOpt()); 

    // Create sensors for this trajectory
    const mapping::TrajectoryBuilderInterface::SensorId rangeSensor{mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, "range"};

    // Create the TrajectoryBuilder (local SLAM)
    int trajId = mapBuilder->AddTrajectoryBuilder({rangeSensor}, trajBuilderOpt.getTrajectoryOpt(), GetLocalSlamResultCallback());
    mapping::TrajectoryBuilderInterface* trajBuilder = mapBuilder->GetTrajectoryBuilder(trajId);
    
    // Generate fake trajectory (WIP: use real trajectory)
    constexpr double duration = 4.;        // Seconds.
    constexpr double timeStep = 0.1;       // Seconds.
    constexpr double travelDistance = 1.2; // Meters.
    const auto measurements = GenerateFakeRangeMeasurements(travelDistance, duration, timeStep);

    // Calculate SLAM with fake point cloud
    for (const auto& measurement : measurements) {
        trajBuilder->AddSensorData(rangeSensor.id, measurement);
    }

    // Compare results
    for(unsigned i = 0; i < gt.size(); i++){
        std::cout << "-----------------------------------" << std::endl;
        std::cout  << localSlamResultPoses[i]<<std::endl;
        std::cout << gt[i] << std::endl;
    }

    // WIP: something is not being closed properly and it gives a segmentation fault
    mapBuilder->FinishTrajectory(trajId);
    mapBuilder->pose_graph()->RunFinalOptimization();
    
    return 1;
}