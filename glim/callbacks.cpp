#include <callbacks.hpp>

// #include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

namespace glim {
CallbackSlot<void(const double, const Eigen::Vector3d&, const Eigen::Vector3d&)> OdometryEstimationCallbacks::on_insert_imu;
}  // namespace glim