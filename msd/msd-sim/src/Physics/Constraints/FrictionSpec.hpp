// Ticket: 0052b_cone_projection_and_linear_algebra
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#pragma once

#include <vector>

namespace msd_sim
{

/// Friction specification for the cone solver (replaces FrictionConeSpec).
/// Lightweight data structure carrying per-contact friction coefficients.
///
/// @ticket 0052b_cone_projection_and_linear_algebra
struct FrictionSpec
{
    int numContacts{0};
    std::vector<double> frictionCoefficients;  // mu per contact, size = numContacts
};

}  // namespace msd_sim
