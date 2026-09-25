#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>

#include "nav2_chauffeur_critic/chauffeur_critic.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

namespace mppi::critics
{
namespace
{

Eigen::ArrayXXf deadbandSquared(const Eigen::ArrayXXf & values, const float deadband)
{
  return (values.abs() - deadband).max(0.0f).square();
}

Eigen::ArrayXf integrateRows(const Eigen::ArrayXXf & values, const float dt)
{
  if (values.cols() == 0) {
    return Eigen::ArrayXf::Zero(values.rows());
  }
  return (values * dt).rowwise().sum();
}

Eigen::ArrayXXf firstDifference(const Eigen::ArrayXXf & values, const float dt)
{
  if (values.cols() < 2) {
    return Eigen::ArrayXXf::Zero(values.rows(), 0);
  }
  return (values.rightCols(values.cols() - 1) -
    values.leftCols(values.cols() - 1)) / dt;
}

Eigen::ArrayXXf secondDifference(const Eigen::ArrayXXf & values, const float dt)
{
  if (values.cols() < 3) {
    return Eigen::ArrayXXf::Zero(values.rows(), 0);
  }
  const auto d1 = firstDifference(values, dt);
  return firstDifference(d1, dt);
}

}  // namespace

void ChauffeurCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1u);
  getParam(weight_, "cost_weight", 1.0f);

  getParam(longitudinal_jerk_weight_, "longitudinal_jerk_weight", 1.0f);
  getParam(lateral_accel_weight_, "lateral_accel_weight", 0.35f);
  getParam(lateral_jerk_weight_, "lateral_jerk_weight", 1.0f);
  getParam(steering_effort_weight_, "steering_effort_weight", 0.10f);
  getParam(steering_rate_weight_, "steering_rate_weight", 0.80f);
  getParam(steering_accel_weight_, "steering_accel_weight", 0.35f);

  getParam(longitudinal_jerk_deadband_, "longitudinal_jerk_deadband", 0.75f);
  getParam(lateral_accel_deadband_, "lateral_accel_deadband", 1.50f);
  getParam(lateral_jerk_deadband_, "lateral_jerk_deadband", 1.00f);
  getParam(steering_effort_deadband_, "steering_effort_deadband", 0.75f);
  getParam(steering_rate_deadband_, "steering_rate_deadband", 0.80f);
  getParam(steering_accel_deadband_, "steering_accel_deadband", 2.00f);
  getParam(min_steering_speed_, "min_steering_speed", 0.25f);
  getParam(max_normalized_steering_, "max_normalized_steering", 1.50f);
  getParam(
    apply_steering_terms_to_non_ackermann_,
    "apply_steering_terms_to_non_ackermann", false);

  min_steering_speed_ = std::max(min_steering_speed_, 1.0e-3f);
  max_normalized_steering_ = std::max(max_normalized_steering_, 1.0f);

  RCLCPP_INFO(
    logger_,
    "ChauffeurCritic loaded: weight=%.3f, jerk db=%.3f m/s^3, lateral db=%.3f m/s^2",
    weight_, longitudinal_jerk_deadband_, lateral_accel_deadband_);
}

void ChauffeurCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const float dt = data.model_dt;
  if (dt <= 0.0f || data.state.vx.cols() == 0) {
    return;
  }

  const auto & vx = data.state.vx;
  const auto & wz = data.state.wz;
  Eigen::ArrayXf comfort_cost = Eigen::ArrayXf::Zero(vx.rows());

  // Longitudinal jerk: j_x = d^2(v_x)/dt^2.
  if (vx.cols() >= 3 && longitudinal_jerk_weight_ > 0.0f) {
    const auto longitudinal_jerk = secondDifference(vx, dt);
    comfort_cost += longitudinal_jerk_weight_ * integrateRows(
      deadbandSquared(longitudinal_jerk, longitudinal_jerk_deadband_), dt);
  }

  // Planar no-slip approximation: lateral acceleration a_y = v_x * omega_z.
  const Eigen::ArrayXXf lateral_accel = vx * wz;
  if (lateral_accel_weight_ > 0.0f) {
    comfort_cost += lateral_accel_weight_ * integrateRows(
      deadbandSquared(lateral_accel, lateral_accel_deadband_), dt);
  }

  if (lateral_accel.cols() >= 2 && lateral_jerk_weight_ > 0.0f) {
    const auto lateral_jerk = firstDifference(lateral_accel, dt);
    comfort_cost += lateral_jerk_weight_ * integrateRows(
      deadbandSquared(lateral_jerk, lateral_jerk_deadband_), dt);
  }

  // Ackermann steering proxy:
  // curvature = omega_z / |v_x|
  // normalized steering = curvature * min_turning_radius
  // |normalized steering| ~= 1 at the motion model's full-lock curvature.
  const auto * ackermann =
    dynamic_cast<const AckermannMotionModel *>(data.motion_model.get());

  if (ackermann != nullptr || apply_steering_terms_to_non_ackermann_) {
    const float min_turning_radius = ackermann != nullptr ?
      std::max(ackermann->getMinTurningRadius(), 1.0e-3f) : 1.0f;

    Eigen::ArrayXXf speed_denominator = vx.abs().max(min_steering_speed_);
    Eigen::ArrayXXf normalized_steering =
      (wz / speed_denominator) * min_turning_radius;
    normalized_steering = normalized_steering
      .max(-max_normalized_steering_)
      .min(max_normalized_steering_);

    if (steering_effort_weight_ > 0.0f) {
      comfort_cost += steering_effort_weight_ * integrateRows(
        deadbandSquared(normalized_steering, steering_effort_deadband_), dt);
    }

    if (normalized_steering.cols() >= 2 && steering_rate_weight_ > 0.0f) {
      const auto steering_rate = firstDifference(normalized_steering, dt);
      comfort_cost += steering_rate_weight_ * integrateRows(
        deadbandSquared(steering_rate, steering_rate_deadband_), dt);
    }

    if (normalized_steering.cols() >= 3 && steering_accel_weight_ > 0.0f) {
      const auto steering_accel = secondDifference(normalized_steering, dt);
      comfort_cost += steering_accel_weight_ * integrateRows(
        deadbandSquared(steering_accel, steering_accel_deadband_), dt);
    }
  }

  comfort_cost *= weight_;
  if (power_ > 1u) {
    comfort_cost = comfort_cost.pow(static_cast<float>(power_));
  }

  data.costs += comfort_cost;
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::ChauffeurCritic, mppi::critics::CriticFunction)
