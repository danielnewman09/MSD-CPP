target_sources(${MSD_SIM_NAME} PRIVATE
    QuaternionConstraint.cpp
    Constraint.cpp
    UnitQuaternionConstraint.cpp
    DistanceConstraint.cpp
    ConstraintSolver.cpp
    ContactConstraint.cpp
    ContactConstraintFactory.cpp
    FrictionConstraint.cpp
    FrictionConeSolver.cpp
    NLoptFrictionSolver.cpp
    PositionCorrector.cpp
    ContactCache.cpp
    ConeProjection.cpp
)
