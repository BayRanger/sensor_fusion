/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/gnss_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class VelocityData {
  public:
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
  
  public:
    static bool SyncDataFromGnssIMU(std::deque<GNSSData>& GNSSData, std::deque<IMUData>& ImuData,std::deque<VelocityData>& SyncedData,double sync_time);
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time, double time_diff);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};
}
#endif