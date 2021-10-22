
#include <cartographer_without_ros/MapBuilderOptionsYaml.h>
#include <cartographer_without_ros/TrajectoryBuilderOptionsYaml.h>
#include <iostream>
#include <vector>

std::vector<transform::Rigid3d> local_slam_result_poses_;
std::vector<transform::Rigid3f> gt;

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
    std::string path = _argv[1];
    MapBuilderOptionsYaml mapBuilderOpt(path);
    TrajectoryBuilderOptionsYaml trajBuilderOpt(path);

    auto mapBuilder = mapping::CreateMapBuilder(mapBuilderOpt.getMapOpt());

    const mapping::TrajectoryBuilderInterface::SensorId kRangeSensorId{mapping::TrajectoryBuilderInterface::SensorId::SensorType::RANGE, "range"};
    int trajId = mapBuilder->AddTrajectoryBuilder({kRangeSensorId}, trajBuilderOpt.getTrajectoryOpt(), GetLocalSlamResultCallback());
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