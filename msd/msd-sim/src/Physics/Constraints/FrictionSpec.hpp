// Ticket: 0052b_cone_projection_and_linear_algebra
// Ticket: 0069_friction_velocity_reversal
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#pragma once

#include <vector>

namespace msd_sim
{

/// Friction specification for the cone solver (replaces FrictionConeSpec).
/// Lightweight data structure carrying per-contact friction coefficients
/// and optional tangent1 lower bounds for sliding mode.
///
/// @ticket 0052b_cone_projection_and_linear_algebra
/// @ticket 0069_friction_velocity_reversal
struct FrictionSpec
{
    int numContacts{0};
    std::vector<double> frictionCoefficients;  // mu per contact, size = numContacts
    std::vector<double> tangent1LowerBounds;   // optional lambda_t1 >= bound per contact (empty = bilateral)
};

}  // namespace msd_sim
