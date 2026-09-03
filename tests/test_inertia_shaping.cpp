#include <gtest/gtest.h>

#include "crisp_controllers/utils/inertia_shaping.hpp"

namespace {

using crisp_controllers::CartesianWrench;
using crisp_controllers::InertiaShaping;
using crisp_controllers::InertiaShapingConfig;

TEST(InertiaShapingTest, DisabledPathsPassEnvironmentWrenchThrough) {
  InertiaShaping shaping;
  const CartesianWrench environment =
    (CartesianWrench() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  const CartesianWrench measured = CartesianWrench::Constant(20.0);

  const auto output = shaping.update(environment, measured);

  EXPECT_TRUE(output.commanded_environment_wrench.isApprox(environment));
  EXPECT_TRUE(output.active_measurement_wrench.isZero());
}

TEST(InertiaShapingTest, ActiveMeasurementAppliesSignFilterAndSeparateScales) {
  InertiaShaping shaping;
  InertiaShapingConfig config;
  config.active_measurement_enabled = true;
  config.active_force_scale = 2.0;
  config.active_torque_scale = 3.0;
  config.active_filter_alpha = 0.5;
  config.measurement_sign = -1.0;
  shaping.configure(config);

  const CartesianWrench measured = CartesianWrench::Ones();
  const auto output = shaping.update(CartesianWrench::Zero(), measured);

  EXPECT_TRUE(output.filtered_measurement_wrench.isApprox(CartesianWrench::Constant(-0.5)));
  EXPECT_TRUE(output.active_measurement_wrench.head<3>().isApprox(Eigen::Vector3d::Constant(-1.0)));
  EXPECT_TRUE(output.active_measurement_wrench.tail<3>().isApprox(Eigen::Vector3d::Constant(-1.5)));
}

TEST(InertiaShapingTest, OneSamplePathUsesPreviousFilteredResidual) {
  InertiaShaping shaping;
  InertiaShapingConfig config;
  config.one_sample_enabled = true;
  config.one_sample_gamma = 0.2;
  config.one_sample_filter_alpha = 0.0;
  config.measurement_sign = 1.0;
  shaping.configure(config);

  CartesianWrench environment = CartesianWrench::Zero();
  CartesianWrench measured = CartesianWrench::Zero();
  environment[0] = 10.0;
  measured[0] = 4.0;

  const auto first = shaping.update(environment, measured);
  EXPECT_DOUBLE_EQ(first.commanded_environment_wrench[0], 10.0);
  EXPECT_DOUBLE_EQ(first.filtered_residual_wrench[0], 6.0);

  environment[0] = 11.0;
  measured[0] = 5.0;
  const auto second = shaping.update(environment, measured);
  EXPECT_DOUBLE_EQ(second.commanded_environment_wrench[0], 12.2);
  EXPECT_DOUBLE_EQ(second.filtered_residual_wrench[0], 6.0);
}

TEST(InertiaShapingTest, OneSampleResidualIsLowPassFiltered) {
  InertiaShaping shaping;
  InertiaShapingConfig config;
  config.one_sample_enabled = true;
  config.one_sample_gamma = 0.25;
  config.one_sample_filter_alpha = 0.5;
  shaping.configure(config);

  CartesianWrench environment = CartesianWrench::Zero();
  environment[2] = 8.0;
  const auto first = shaping.update(environment, CartesianWrench::Zero());
  EXPECT_DOUBLE_EQ(first.commanded_environment_wrench[2], 8.0);
  EXPECT_DOUBLE_EQ(first.filtered_residual_wrench[2], 4.0);

  const auto second = shaping.update(environment, CartesianWrench::Zero());
  EXPECT_DOUBLE_EQ(second.commanded_environment_wrench[2], 9.0);
  EXPECT_DOUBLE_EQ(second.filtered_residual_wrench[2], 6.0);
}

TEST(InertiaShapingTest, ResetClearsPreviousSampleContribution) {
  InertiaShaping shaping;
  InertiaShapingConfig config;
  config.one_sample_enabled = true;
  config.one_sample_gamma = 0.5;
  config.one_sample_filter_alpha = 0.0;
  shaping.configure(config);

  const CartesianWrench environment = CartesianWrench::Ones();
  static_cast<void>(shaping.update(environment, CartesianWrench::Zero()));
  shaping.reset();
  const auto output = shaping.update(environment, CartesianWrench::Zero());

  EXPECT_TRUE(output.commanded_environment_wrench.isApprox(environment));
}

}  // namespace
