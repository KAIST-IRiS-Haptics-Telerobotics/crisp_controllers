#pragma once

#include <cstddef>

#include <Eigen/Dense>  // NOLINT(build/include_order)

#include "crisp_controllers/utils/fiters.hpp"

namespace crisp_controllers {

using CartesianWrench = Eigen::Matrix<double, 6, 1>;

/** Parameters for the local measured-wrench inertia-shaping terms. */
struct InertiaShapingConfig {
  bool active_measurement_enabled{false};
  double active_force_scale{2.0};
  double active_torque_scale{3.0};
  double active_filter_alpha{0.99};
  bool active_zero_on_enable{false};
  std::size_t active_zero_samples{1000};

  bool one_sample_enabled{false};
  double one_sample_gamma{0.1};
  double one_sample_filter_alpha{0.99};

  // Set to +1 or -1 after the static sign-identification test.
  double measurement_sign{1.0};
};

/** Wrench-space result produced for one controller cycle. */
struct InertiaShapingOutput {
  CartesianWrench commanded_environment_wrench{CartesianWrench::Zero()};
  CartesianWrench active_measurement_wrench{CartesianWrench::Zero()};
  CartesianWrench filtered_measurement_wrench{CartesianWrench::Zero()};
  CartesianWrench filtered_residual_wrench{CartesianWrench::Zero()};
  CartesianWrench active_measurement_bias{CartesianWrench::Zero()};
  bool active_measurement_ready{true};
};

/**
 * Stateful wrench-space implementation of the two IRIS inertia-compensation experiments.
 *
 * The active-measurement path implements
 *   w_bar[k] = alpha w_bar[k-1] + (1-alpha) s_h w_h[k]
 * and applies independent force/torque scales.
 *
 * The one-sample path implements
 *   u[k] = f_e[k] + gamma d_bar[k-1]
 *   d_bar[k] = LPF(f_e[k] - s_h w_h[k]).
 * The state update deliberately happens after u[k] is formed so that the residual is exactly
 * one controller sample old.
 */
class InertiaShaping {
public:
  void configure(const InertiaShapingConfig & config) {
    const bool sign_changed = config_.measurement_sign != config.measurement_sign;
    const bool active_just_enabled =
      !config_.active_measurement_enabled && config.active_measurement_enabled;
    const bool one_sample_just_enabled = !config_.one_sample_enabled && config.one_sample_enabled;
    const bool zeroing_changed =
      config_.active_zero_on_enable != config.active_zero_on_enable ||
      config_.active_zero_samples != config.active_zero_samples;
    if (sign_changed || active_just_enabled || one_sample_just_enabled || zeroing_changed) {
      reset();
    }
    config_ = config;
  }

  [[nodiscard]] InertiaShapingOutput
  update(const CartesianWrench & environment_wrench, const CartesianWrench & measured_wrench) {
    InertiaShapingOutput output;
    output.commanded_environment_wrench = environment_wrench;

    const CartesianWrench aligned_measurement = config_.measurement_sign * measured_wrench;
    CartesianWrench active_measurement = aligned_measurement;
    if (
      config_.active_measurement_enabled && config_.active_zero_on_enable &&
      active_zero_count_ < config_.active_zero_samples) {
      ++active_zero_count_;
      active_measurement_bias_ +=
        (aligned_measurement - active_measurement_bias_) / static_cast<double>(active_zero_count_);
      output.active_measurement_bias = active_measurement_bias_;
      output.active_measurement_ready = false;
    } else {
      if (config_.active_measurement_enabled && config_.active_zero_on_enable) {
        active_measurement -= active_measurement_bias_;
      }
      filtered_measurement_ = exponential_moving_average(
        filtered_measurement_, active_measurement, config_.active_filter_alpha);
      output.filtered_measurement_wrench = filtered_measurement_;
      output.active_measurement_bias = active_measurement_bias_;
      output.active_measurement_ready = true;

      if (config_.active_measurement_enabled) {
        output.active_measurement_wrench.head<3>() =
          config_.active_force_scale * filtered_measurement_.head<3>();
        output.active_measurement_wrench.tail<3>() =
          config_.active_torque_scale * filtered_measurement_.tail<3>();
      }
    }

    // Use the residual retained from the preceding controller cycle.
    if (config_.one_sample_enabled) {
      output.commanded_environment_wrench += config_.one_sample_gamma * filtered_residual_;
    }

    const CartesianWrench residual = environment_wrench - aligned_measurement;
    filtered_residual_ =
      exponential_moving_average(filtered_residual_, residual, config_.one_sample_filter_alpha);
    output.filtered_residual_wrench = filtered_residual_;

    return output;
  }

  [[nodiscard]] InertiaShapingOutput passthrough(const CartesianWrench & environment_wrench) const {
    InertiaShapingOutput output;
    output.commanded_environment_wrench = environment_wrench;
    return output;
  }

  void reset() {
    filtered_measurement_.setZero();
    filtered_residual_.setZero();
    active_measurement_bias_.setZero();
    active_zero_count_ = 0;
  }

private:
  InertiaShapingConfig config_{};
  CartesianWrench filtered_measurement_{CartesianWrench::Zero()};
  CartesianWrench filtered_residual_{CartesianWrench::Zero()};
  CartesianWrench active_measurement_bias_{CartesianWrench::Zero()};
  std::size_t active_zero_count_{0};
};

}  // namespace crisp_controllers
