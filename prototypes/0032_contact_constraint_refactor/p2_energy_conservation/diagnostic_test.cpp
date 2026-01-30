// Diagnostic Test: Root-cause analysis for P2 energy injection
//
// Traces the exact computation at the first bounce to identify
// where energy is being injected in the constraint-based approach.

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

constexpr double mass = 1.0;
constexpr double radius = 1.0;
constexpr double gravity = 9.81;
constexpr double dt = 0.016;
constexpr double restitution = 0.8;
constexpr double velocity_threshold = 0.5;
constexpr double slop = 0.01;
constexpr double correction_factor = 0.8;

struct State {
    double z;
    double vz;
    double ke() const { return 0.5 * mass * vz * vz; }
    double pe() const { return mass * gravity * z; }
    double energy() const { return ke() + pe(); }
};

void printState(const std::string& label, const State& s) {
    std::cout << "  " << std::setw(30) << std::left << label
              << " z=" << std::fixed << std::setprecision(6) << s.z
              << " vz=" << std::setprecision(6) << s.vz
              << " KE=" << std::setprecision(4) << s.ke()
              << " PE=" << std::setprecision(4) << s.pe()
              << " E=" << std::setprecision(4) << s.energy()
              << "\n";
}

// ---- Test 1: Trace both approaches at the first bounce ----
void traceFirstBounce() {
    std::cout << "=== Test 1: Trace First Bounce Step-by-Step ===\n\n";

    // Run until just before first contact
    State impulse{5.0, 0.0};
    State constraint{5.0, 0.0};

    int bounceFrame = -1;

    for (int frame = 0; frame < 100; ++frame) {
        // Save pre-step states
        State imp_pre = impulse;
        State con_pre = constraint;

        // Apply gravity to both
        impulse.vz -= gravity * dt;
        constraint.vz -= gravity * dt;

        // Integrate positions
        impulse.z += impulse.vz * dt;
        constraint.z += constraint.vz * dt;

        double imp_gap = impulse.z - radius;
        double con_gap = constraint.z - radius;

        if (imp_gap < 0.0 && bounceFrame < 0) {
            bounceFrame = frame;

            std::cout << "First contact at frame " << frame << ":\n\n";

            // ============ IMPULSE-BASED ============
            std::cout << "--- IMPULSE-BASED ---\n";
            printState("Before gravity:", imp_pre);

            State imp_after_grav{imp_pre.z, imp_pre.vz - gravity * dt};
            printState("After gravity:", imp_after_grav);

            State imp_after_integ{imp_after_grav.z + imp_after_grav.vz * dt, imp_after_grav.vz};
            printState("After integration:", imp_after_integ);

            double imp_gap_val = imp_after_integ.z - radius;
            double imp_v_rel_n = imp_after_integ.vz;  // dot with (0,0,1) = vz
            std::cout << "  gap = " << imp_gap_val << " m  (penetrating: "
                      << (imp_gap_val < 0 ? "YES" : "NO") << ")\n";
            std::cout << "  v_rel_n = " << imp_v_rel_n << " m/s\n";
            std::cout << "  threshold check: " << imp_v_rel_n << " < -"
                      << velocity_threshold << " = "
                      << (imp_v_rel_n < -velocity_threshold ? "YES (apply restitution)" : "NO") << "\n";

            // The impulse approach:
            // effective_mass = 1/m = 1.0
            // lambda = -(1+e) * v_rel_n / effective_mass
            // v += lambda * n / m = lambda / m (since n=(0,0,1))
            double imp_effective_mass = 1.0 / mass;
            double imp_lambda = -(1.0 + restitution) * imp_v_rel_n / imp_effective_mass;
            double imp_v_change = imp_lambda / mass;
            double imp_v_new = imp_after_integ.vz + imp_v_change;

            std::cout << "  effective_mass = 1/m = " << imp_effective_mass << "\n";
            std::cout << "  lambda = -(1+e)*v_rel_n / eff_mass = -("
                      << (1+restitution) << ")*(" << imp_v_rel_n << ") / " << imp_effective_mass
                      << " = " << imp_lambda << "\n";
            std::cout << "  v_change = lambda/m = " << imp_v_change << "\n";
            std::cout << "  v_new = " << imp_after_integ.vz << " + " << imp_v_change
                      << " = " << imp_v_new << "\n";

            State imp_after_impulse{imp_after_integ.z, imp_v_new};
            printState("After impulse:", imp_after_impulse);

            // Position correction
            double imp_correction = std::max(0.0, -imp_gap_val - slop) * correction_factor;
            State imp_final{imp_after_impulse.z + imp_correction, imp_after_impulse.vz};
            std::cout << "  position correction = " << imp_correction << " m\n";
            printState("After correction:", imp_final);
            std::cout << "  Energy change: " << (imp_final.energy() - imp_pre.energy()) << " J\n\n";

            impulse = imp_final;

            // ============ CONSTRAINT-BASED ============
            std::cout << "--- CONSTRAINT-BASED ---\n";
            printState("Before gravity:", con_pre);

            State con_after_grav{con_pre.z, con_pre.vz - gravity * dt};
            printState("After gravity:", con_after_grav);

            State con_after_integ{con_after_grav.z + con_after_grav.vz * dt, con_after_grav.vz};
            printState("After integration:", con_after_integ);

            double con_gap_val = con_after_integ.z - radius;
            double con_gap_dot = con_after_integ.vz;
            std::cout << "  gap C(q) = " << con_gap_val << " m\n";
            std::cout << "  gap_dot Cdot = " << con_gap_dot << " m/s\n";
            std::cout << "  threshold check: " << con_gap_dot << " < -"
                      << velocity_threshold << " = "
                      << (con_gap_dot < -velocity_threshold ? "YES (apply restitution)" : "NO") << "\n";

            // The constraint approach:
            // desired_gap_dot = -(1+e) * gap_dot  (BUG candidate!)
            // delta_gap_dot = desired_gap_dot - gap_dot
            // lambda = delta_gap_dot * mass * effective_mass
            double con_effective_mass = 1.0 / mass;
            double con_desired_gap_dot = 0.0;
            if (con_gap_dot < -velocity_threshold) {
                con_desired_gap_dot = -(1.0 + restitution) * con_gap_dot;
            } else if (con_gap_dot < 0.0) {
                con_desired_gap_dot = 0.0;
            }
            double con_delta_gap_dot = con_desired_gap_dot - con_gap_dot;
            double con_lambda = con_delta_gap_dot * mass * con_effective_mass;
            double con_v_change = con_lambda / mass;
            double con_v_new = con_after_integ.vz + con_v_change;

            std::cout << "  desired_gap_dot = -(1+e)*gap_dot = -("
                      << (1+restitution) << ")*(" << con_gap_dot << ") = "
                      << con_desired_gap_dot << "\n";
            std::cout << "  delta_gap_dot = desired - current = "
                      << con_desired_gap_dot << " - (" << con_gap_dot << ") = "
                      << con_delta_gap_dot << "\n";
            std::cout << "  lambda = delta * m * eff_mass = "
                      << con_delta_gap_dot << " * " << mass << " * " << con_effective_mass
                      << " = " << con_lambda << "\n";
            std::cout << "  v_change = lambda/m = " << con_v_change << "\n";
            std::cout << "  v_new = " << con_after_integ.vz << " + " << con_v_change
                      << " = " << con_v_new << "\n";

            State con_after_impulse{con_after_integ.z, con_v_new};
            printState("After constraint impulse:", con_after_impulse);

            // Position correction
            double con_correction = std::max(0.0, -con_gap_val - slop) * correction_factor;
            State con_final{con_after_impulse.z + con_correction, con_after_impulse.vz};
            std::cout << "  position correction = " << con_correction << " m\n";
            printState("After correction:", con_final);
            std::cout << "  Energy change: " << (con_final.energy() - con_pre.energy()) << " J\n\n";

            constraint = con_final;

            // ============ COMPARISON ============
            std::cout << "--- COMPARISON ---\n";
            std::cout << "  Impulse v_new:    " << imp_v_new << " m/s\n";
            std::cout << "  Constraint v_new: " << con_v_new << " m/s\n";
            std::cout << "  DIFFERENCE:       " << (con_v_new - imp_v_new) << " m/s\n";
            std::cout << "  Impulse lambda:    " << imp_lambda << "\n";
            std::cout << "  Constraint lambda: " << con_lambda << "\n\n";

            // ============ WHAT SHOULD HAPPEN ============
            std::cout << "--- EXPECTED (from math formulation) ---\n";
            std::cout << "  The restitution formula gives target post-bounce velocity:\n";
            std::cout << "    v_target = -e * v_pre = -" << restitution << " * ("
                      << con_gap_dot << ") = " << (-restitution * con_gap_dot) << " m/s\n";
            std::cout << "  The impulse needed:\n";
            std::cout << "    J = m * (v_target - v_pre) = " << mass << " * ("
                      << (-restitution * con_gap_dot) << " - " << con_gap_dot << ")\n";
            std::cout << "    J = m * (1 + e) * |v_pre| = "
                      << mass * (1 + restitution) * std::abs(con_gap_dot) << "\n";
            std::cout << "    v_new = v_pre + J/m = " << con_gap_dot << " + "
                      << ((1 + restitution) * std::abs(con_gap_dot))
                      << " = " << (con_gap_dot + (1 + restitution) * std::abs(con_gap_dot)) << "\n";

            double v_correct = -restitution * con_gap_dot;
            std::cout << "  Correct v_new should be: " << v_correct << " m/s\n";
            std::cout << "  Impulse-based produces:  " << imp_v_new << " m/s  ("
                      << (std::abs(imp_v_new - v_correct) < 0.01 ? "CORRECT" : "WRONG") << ")\n";
            std::cout << "  Constraint-based produces: " << con_v_new << " m/s  ("
                      << (std::abs(con_v_new - v_correct) < 0.01 ? "CORRECT" : "WRONG") << ")\n\n";

            continue;  // Skip normal integration for this frame
        }

        // Normal frames (no contact)
        // Already integrated above
    }
}

// ---- Test 2: Analyze the restitution formula bug ----
void analyzeRestitutionBug() {
    std::cout << "\n=== Test 2: Restitution Formula Analysis ===\n\n";

    double v_pre = -8.0;  // Approaching at 8 m/s (typical first bounce)
    double e = 0.8;

    std::cout << "Pre-collision velocity: v = " << v_pre << " m/s\n";
    std::cout << "Restitution coefficient: e = " << e << "\n\n";

    // Correct physics: v_post = -e * v_pre
    double v_correct = -e * v_pre;
    std::cout << "CORRECT post-velocity: v_post = -e * v_pre = " << v_correct << " m/s\n";
    std::cout << "  Impulse needed: J = m*(v_post - v_pre) = "
              << mass * (v_correct - v_pre) << " N·s\n\n";

    // Impulse-based code:
    // lambda = -(1+e)*v_rel_n / effective_mass  where effective_mass = 1/m
    // v_new = v + lambda/m
    double eff_mass = 1.0 / mass;
    double imp_lambda = -(1.0 + e) * v_pre / eff_mass;
    double imp_v_new = v_pre + imp_lambda / mass;
    std::cout << "IMPULSE-BASED code:\n";
    std::cout << "  lambda = -(1+e)*v / eff_mass = -" << (1+e) << "*" << v_pre
              << "/" << eff_mass << " = " << imp_lambda << "\n";
    std::cout << "  v_new = v + lambda/m = " << v_pre << " + " << (imp_lambda/mass)
              << " = " << imp_v_new << " m/s\n";
    std::cout << "  " << (std::abs(imp_v_new - v_correct) < 0.001 ? "CORRECT" : "WRONG") << "\n\n";

    // Constraint-based code:
    // desired_gap_dot = -(1+e)*gap_dot  where gap_dot = vz
    // delta_gap_dot = desired - current
    // lambda = delta * m * eff_mass
    // v_new = v + lambda/m
    double con_desired = -(1.0 + e) * v_pre;
    double con_delta = con_desired - v_pre;
    double con_lambda = con_delta * mass * eff_mass;
    double con_v_new = v_pre + con_lambda / mass;
    std::cout << "CONSTRAINT-BASED code:\n";
    std::cout << "  desired_gap_dot = -(1+e)*v = -" << (1+e) << "*" << v_pre
              << " = " << con_desired << "\n";
    std::cout << "  delta = desired - current = " << con_desired << " - (" << v_pre
              << ") = " << con_delta << "\n";
    std::cout << "  lambda = delta * m * eff = " << con_delta << "*" << mass << "*"
              << eff_mass << " = " << con_lambda << "\n";
    std::cout << "  v_new = v + lambda/m = " << v_pre << " + " << (con_lambda/mass)
              << " = " << con_v_new << " m/s\n";
    std::cout << "  " << (std::abs(con_v_new - v_correct) < 0.001 ? "CORRECT" : "WRONG") << "\n\n";

    // Algebraic analysis
    std::cout << "--- ALGEBRAIC ANALYSIS ---\n\n";
    std::cout << "Impulse-based:\n";
    std::cout << "  lambda = -(1+e)*v / (1/m) = -(1+e)*v*m\n";
    std::cout << "  v_new  = v + lambda/m = v + (-(1+e)*v*m)/m = v - (1+e)*v = v*(1 - 1 - e) = -e*v\n";
    std::cout << "  Result: v_new = -e*v = " << (-e * v_pre) << " ✓ CORRECT\n\n";

    std::cout << "Constraint-based:\n";
    std::cout << "  desired = -(1+e)*v\n";
    std::cout << "  delta   = desired - v = -(1+e)*v - v = -(2+e)*v\n";
    std::cout << "  lambda  = delta * m * (1/m) = delta = -(2+e)*v\n";
    std::cout << "  v_new   = v + lambda/m = v + delta = v + (-(2+e)*v) = v*(1 - 2 - e) = -(1+e)*v\n";
    std::cout << "  Result: v_new = -(1+e)*v = " << (-(1+e) * v_pre) << " ✗ WRONG!\n\n";

    std::cout << "THE BUG: The desired velocity is set to -(1+e)*v, but the correct\n";
    std::cout << "target is -e*v. The code confuses the constraint RHS (which already\n";
    std::cout << "includes the pre-velocity contribution) with the desired post-velocity.\n\n";

    // The fix
    double fix_desired = -e * v_pre;
    double fix_delta = fix_desired - v_pre;
    double fix_lambda = fix_delta * mass * eff_mass;
    double fix_v_new = v_pre + fix_lambda / mass;
    std::cout << "FIXED constraint-based (desired = -e*v):\n";
    std::cout << "  desired = -e*v = " << fix_desired << "\n";
    std::cout << "  delta = " << fix_desired << " - " << v_pre << " = " << fix_delta << "\n";
    std::cout << "  lambda = " << fix_lambda << "\n";
    std::cout << "  v_new = " << v_pre << " + " << (fix_lambda/mass) << " = " << fix_v_new << "\n";
    std::cout << "  " << (std::abs(fix_v_new - v_correct) < 0.001 ? "CORRECT" : "WRONG") << "\n\n";

    // Energy comparison
    double ke_pre = 0.5 * mass * v_pre * v_pre;
    double ke_correct = 0.5 * mass * v_correct * v_correct;
    double ke_buggy = 0.5 * mass * con_v_new * con_v_new;
    std::cout << "--- ENERGY COMPARISON ---\n";
    std::cout << "  KE before:           " << ke_pre << " J\n";
    std::cout << "  KE correct (−e*v):   " << ke_correct << " J  (ratio: "
              << (ke_correct/ke_pre) << ", expected e²=" << (e*e) << ")\n";
    std::cout << "  KE buggy (−(1+e)*v): " << ke_buggy << " J  (ratio: "
              << (ke_buggy/ke_pre) << ", expected (1+e)²=" << ((1+e)*(1+e)) << ")\n";
    std::cout << "  Energy change correct: " << (ke_correct - ke_pre) << " J (dissipation)\n";
    std::cout << "  Energy change buggy:   " << (ke_buggy - ke_pre) << " J (INJECTION!)\n";
}

// ---- Test 3: Verify the fix works end-to-end ----
void verifyFix() {
    std::cout << "\n=== Test 3: End-to-End Verification of Fix ===\n\n";

    // Run corrected constraint simulation
    double z = 5.0;
    double vz = 0.0;
    double initial_energy = mass * gravity * z;

    std::cout << "Running corrected constraint simulation (desired = -e*v)...\n\n";

    int bounces = 0;
    double max_energy = initial_energy;
    double min_energy = initial_energy;
    bool energy_monotonic = true;
    double prev_energy = initial_energy;

    for (int frame = 0; frame < 1000; ++frame) {
        vz -= gravity * dt;
        z += vz * dt;

        double gap = z - radius;

        if (gap < 0.0) {
            double gap_dot = vz;

            if (gap_dot < -velocity_threshold) {
                // FIX: desired = -e * gap_dot (not -(1+e) * gap_dot)
                double desired_gap_dot = -restitution * gap_dot;
                double delta = desired_gap_dot - gap_dot;
                vz += delta;  // simplified since m * eff_mass / m = 1
                bounces++;
            } else if (gap_dot < 0.0) {
                vz = 0.0;
            }

            double correction = std::max(0.0, -gap - slop) * correction_factor;
            z += correction;
        }

        double energy = 0.5 * mass * vz * vz + mass * gravity * z;
        if (energy > prev_energy + 0.1) {
            energy_monotonic = false;
        }
        max_energy = std::max(max_energy, energy);
        min_energy = std::min(min_energy, energy);
        prev_energy = energy;

        if (frame % 100 == 0 || frame == 999) {
            std::cout << "  Frame " << std::setw(4) << frame
                      << ": z=" << std::fixed << std::setprecision(4) << z
                      << " vz=" << std::setprecision(4) << vz
                      << " E=" << std::setprecision(4) << energy << " J\n";
        }
    }

    std::cout << "\n  Bounces: " << bounces << "\n";
    std::cout << "  Initial energy: " << initial_energy << " J\n";
    std::cout << "  Final energy: " << prev_energy << " J\n";
    std::cout << "  Energy change: " << (prev_energy - initial_energy) << " J\n";
    std::cout << "  Energy monotonic: " << (energy_monotonic ? "YES" : "NO") << "\n";
    std::cout << "  Max energy: " << max_energy << " J\n";
    std::cout << "  Criterion 1 (energy decreases): "
              << ((prev_energy < initial_energy) ? "PASS" : "FAIL") << "\n";
    std::cout << "  Criterion 2 (monotonic): "
              << (energy_monotonic ? "PASS" : "FAIL") << "\n";
}

int main() {
    std::cout << std::fixed;

    traceFirstBounce();
    analyzeRestitutionBug();
    verifyFix();

    std::cout << "\n=== ROOT CAUSE SUMMARY ===\n\n";
    std::cout << "The constraint-based code at main.cpp line 161 computes:\n";
    std::cout << "  desired_gap_dot = -(1 + e) * gap_dot\n\n";
    std::cout << "This is the CONSTRAINT RHS (the b vector in A*lambda = b),\n";
    std::cout << "NOT the desired post-collision velocity.\n\n";
    std::cout << "The code then computes delta = desired - current, which gives:\n";
    std::cout << "  delta = -(1+e)*v - v = -(2+e)*v\n";
    std::cout << "  v_new = v + delta = -(1+e)*v\n\n";
    std::cout << "This produces |v_new| = (1+e)*|v_pre| > |v_pre| for e > 0,\n";
    std::cout << "which INJECTS energy instead of dissipating it.\n\n";
    std::cout << "The correct post-collision velocity is:\n";
    std::cout << "  v_target = -e * v_pre\n";
    std::cout << "  delta = -e*v - v = -(1+e)*v\n";
    std::cout << "  v_new = v + delta = -e*v\n\n";
    std::cout << "This is exactly what the impulse-based code computes correctly.\n";

    return 0;
}
