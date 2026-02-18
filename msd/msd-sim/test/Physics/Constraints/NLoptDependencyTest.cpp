// Ticket: 0068a_nlopt_conan_dependency
// Purpose: Verify NLopt dependency is correctly linked and headers are accessible

#include <gtest/gtest.h>
#include <nlopt.hpp>

namespace msd_sim {

// Minimal test to verify NLopt is available and linkable
TEST(NLoptDependencyTest, HeaderIncludesSuccessfully)
{
  // Just verify the header compiles and basic types are available
  // This test will be removed after NLoptFrictionSolver is implemented (0068b)

  // Verify NLopt algorithm enum is accessible
  nlopt::algorithm algo = nlopt::LD_SLSQP;
  EXPECT_EQ(algo, nlopt::LD_SLSQP);

  // Verify NLopt result enum is accessible
  nlopt::result res = nlopt::SUCCESS;
  EXPECT_EQ(res, nlopt::SUCCESS);
}

// Verify we can create an NLopt optimizer object
TEST(NLoptDependencyTest, CanCreateOptimizer)
{
  // Create a simple 2D optimization problem to verify linkage
  nlopt::opt optimizer{nlopt::LD_SLSQP, 2};

  // Verify basic configuration
  EXPECT_EQ(optimizer.get_algorithm(), nlopt::LD_SLSQP);
  EXPECT_EQ(optimizer.get_dimension(), 2);

  // Set tolerance (verify setter compiles)
  optimizer.set_ftol_rel(1e-6);

  // Note: We don't actually solve anything here - just verify the library links
}

}  // namespace msd_sim
