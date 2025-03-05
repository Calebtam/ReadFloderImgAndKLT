#pragma once

#include <map>
#include <callback_slot.hpp>
#include <Eigen/Core>

namespace glim {
/**
 * @brief Odometry estimation-related callbacks
 *
 */
struct OdometryEstimationCallbacks {

  /**
   * @brief IMU input callback
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  static CallbackSlot<void(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel)> on_insert_imu;
};

}  // namespace glim