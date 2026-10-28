/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <limits>

#include <duatic_controllers/force_torque_broadcaster.hpp>

using duatic::controllers::solve_wrench;

namespace
{
constexpr double kLambda = 1e-6;
const Eigen::Matrix<double, 6, 1> kWrench =
    (Eigen::Matrix<double, 6, 1>() << 1.0, -2.0, 3.0, 0.1, -0.2, 0.3).finished();

// 9 joint columns, the first 6 of them an invertible Jacobian, the rest unsupported and zero
Eigen::Matrix<double, 6, Eigen::Dynamic> make_jacobian()
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> J = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, 9);
  J.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity() + 0.1 * Eigen::Matrix<double, 6, 6>::Ones();
  return J;
}
}  // namespace

class SolveWrenchTest : public testing::TestWithParam<bool>
{
};

TEST_P(SolveWrenchTest, RecoversWrenchFromJointTorques)
{
  const auto J = make_jacobian();
  const Eigen::VectorXd tau_ext = J.transpose() * kWrench;

  const auto wrench = solve_wrench(J, tau_ext, kLambda, GetParam());

  ASSERT_TRUE(wrench.has_value());
  EXPECT_TRUE(wrench->isApprox(kWrench, 1e-4));
}

TEST_P(SolveWrenchTest, RejectsNonFiniteJacobian)
{
  auto J = make_jacobian();
  const Eigen::VectorXd tau_ext = J.transpose() * kWrench;
  J.col(8).setConstant(std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(solve_wrench(J, tau_ext, kLambda, GetParam()).has_value());
}

TEST_P(SolveWrenchTest, RejectsNonFiniteTorques)
{
  const auto J = make_jacobian();
  Eigen::VectorXd tau_ext = J.transpose() * kWrench;
  tau_ext(2) = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(solve_wrench(J, tau_ext, kLambda, GetParam()).has_value());
}

INSTANTIATE_TEST_SUITE_P(Solvers, SolveWrenchTest, testing::Values(true, false),
                         [](const testing::TestParamInfo<bool>& info) { return info.param ? "Svd" : "Dls"; });
