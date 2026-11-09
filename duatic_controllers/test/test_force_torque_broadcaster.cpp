/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <limits>

#include <duatic_controllers/force_torque_broadcaster.hpp>

using duatic::controllers::WrenchSolver;

namespace
{
std::atomic<bool> g_count_mallocs{ false };
std::atomic<int> g_malloc_count{ 0 };
}  // namespace

// Interposes malloc for the whole test binary, including the controller library, to count heap allocations
extern "C" void* __libc_malloc(std::size_t size);
extern "C" void* malloc(std::size_t size)
{
  if (g_count_mallocs) {
    ++g_malloc_count;
  }
  return __libc_malloc(size);
}

namespace
{
constexpr Eigen::Index kNv = 9;
constexpr double kLambda = 1e-6;
const WrenchSolver::Wrench kWrench = (WrenchSolver::Wrench() << 1.0, -2.0, 3.0, 0.1, -0.2, 0.3).finished();

// The first 6 joint columns form an invertible Jacobian, the rest are unsupported and zero
WrenchSolver::Jacobian make_jacobian()
{
  WrenchSolver::Jacobian J = WrenchSolver::Jacobian::Zero(6, kNv);
  J.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity() + 0.1 * Eigen::Matrix<double, 6, 6>::Ones();
  return J;
}
}  // namespace

class WrenchSolverTest : public testing::TestWithParam<bool>
{
protected:
  WrenchSolver solver_{ kNv };
  WrenchSolver::Wrench wrench_ = WrenchSolver::Wrench::Zero();
};

TEST_P(WrenchSolverTest, RecoversWrenchFromJointTorques)
{
  const auto J = make_jacobian();
  const Eigen::VectorXd tau_ext = J.transpose() * kWrench;

  ASSERT_TRUE(solver_.solve(J, tau_ext, kLambda, GetParam(), wrench_));
  EXPECT_TRUE(wrench_.isApprox(kWrench, 1e-4));
}

TEST_P(WrenchSolverTest, RejectsNonFiniteJacobian)
{
  auto J = make_jacobian();
  const Eigen::VectorXd tau_ext = J.transpose() * kWrench;
  J.col(8).setConstant(std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(solver_.solve(J, tau_ext, kLambda, GetParam(), wrench_));
}

TEST_P(WrenchSolverTest, RejectsNonFiniteTorques)
{
  const auto J = make_jacobian();
  Eigen::VectorXd tau_ext = J.transpose() * kWrench;
  tau_ext(2) = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(solver_.solve(J, tau_ext, kLambda, GetParam(), wrench_));
}

TEST_P(WrenchSolverTest, DoesNotAllocateWhenSolving)
{
  const auto J = make_jacobian();
  const Eigen::VectorXd tau_ext = J.transpose() * kWrench;

  // Checks that the counter sees allocations made from inside the controller library
  g_malloc_count = 0;
  g_count_mallocs = true;
  const WrenchSolver probe{ kNv };
  g_count_mallocs = false;
  ASSERT_GT(g_malloc_count, 0);

  g_malloc_count = 0;
  g_count_mallocs = true;
  const bool solved = solver_.solve(J, tau_ext, kLambda, GetParam(), wrench_);
  g_count_mallocs = false;

  ASSERT_TRUE(solved);
  EXPECT_EQ(g_malloc_count, 0);
}

INSTANTIATE_TEST_SUITE_P(Solvers, WrenchSolverTest, testing::Values(true, false),
                         [](const testing::TestParamInfo<bool>& info) { return info.param ? "Svd" : "Dls"; });
