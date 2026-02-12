// Ticket: vector_datatype_refactor
// Convenience header for all vector types

#ifndef VECTOR_TYPES_HPP
#define VECTOR_TYPES_HPP

/**
 * @file VectorTypes.hpp
 * @brief Convenience header including all vector data types
 *
 * This header provides a single include point for all msd-sim vector types.
 * Includes both base templates and concrete wrapper types.
 */

// Base templates
#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec4DBase.hpp"

// Formatter bases
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-sim/src/DataTypes/Vec4FormatterBase.hpp"

// Concrete wrapper types
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/DataTypes/Vector4D.hpp"

// Domain-specific types (use base templates)
#include "msd-sim/src/DataTypes/Acceleration.hpp"
#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/ForceVector.hpp"
#include "msd-sim/src/DataTypes/TorqueVector.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"

#endif  // VECTOR_TYPES_HPP
