// Ticket: 0056f_threejs_overlays
// Constraint force, friction force, and gravity arrow visualization

import * as THREE from 'three';

export class ForceOverlay {
    constructor(scene, metadata) {
        this.scene = scene;
        this.metadata = metadata;
        this.constraintEnabled = false;
        this.gravityEnabled = false;
        this.frictionEnabled = false;

        // Overlay objects
        this.constraintArrows = [];  // GREEN arrows for normal constraint forces
        this.frictionArrows = [];    // ORANGE arrows for friction forces
        this.gravityArrows = new Map();  // BLUE arrows for gravity (per body, constant)

        // Configuration
        this.constraintScale = 0.02;  // Scale: Newtons -> visual length
        this.minForce = 0.1;  // Threshold for displaying forces (Newtons)
        this.gravityMagnitude = 9.81;  // m/s^2 (from MSD-CPP gravity constant)
        this.gravityScale = 0.1;  // Visual scale for gravity arrows
    }

    /**
     * Enable/disable constraint force visualization
     * @param {boolean} enabled
     */
    setConstraintEnabled(enabled) {
        this.constraintEnabled = enabled;
        if (!enabled) {
            this.clearConstraintArrows();
        }
    }

    /**
     * Enable/disable gravity visualization
     * @param {boolean} enabled
     */
    setGravityEnabled(enabled) {
        this.gravityEnabled = enabled;
        if (enabled) {
            this.createGravityArrows();
        } else {
            this.clearGravityArrows();
        }
    }

    /**
     * Enable/disable friction force visualization
     * @param {boolean} enabled
     */
    setFrictionEnabled(enabled) {
        this.frictionEnabled = enabled;
        if (!enabled) {
            this.clearFrictionArrows();
        }
    }

    /**
     * Update constraint and friction force arrows from frame data.
     * Uses normal_lambda (actual solver force in Newtons) for arrow scaling
     * when friction_constraints are available, falls back to penetration depth.
     * @param {Object} frameData - Frame data with states, collisions, and friction_constraints
     */
    update(frameData) {
        // Clear previous arrows
        this.clearConstraintArrows();
        this.clearFrictionArrows();

        if (!frameData) return;

        // Build a lookup from (body_a_id, body_b_id) -> normal_lambda from friction constraints
        const lambdaMap = new Map();
        if (frameData.friction_constraints) {
            frameData.friction_constraints.forEach(fc => {
                const key = `${fc.body_a_id}_${fc.body_b_id}`;
                // Sum lambdas for the same pair (multiple contact points)
                lambdaMap.set(key, (lambdaMap.get(key) || 0) + fc.normal_lambda);
            });
        }

        // Normal constraint arrows (GREEN)
        if (this.constraintEnabled && frameData.collisions) {
            // Build body position lookup for computing contact points from lever arms
            const bodyPositions = new Map();
            if (frameData.states) {
                frameData.states.forEach(s => {
                    bodyPositions.set(s.body_id, s.position);
                });
            }

            frameData.collisions.forEach(collision => {
                const normal = new THREE.Vector3(
                    collision.normal.x,
                    collision.normal.y,
                    collision.normal.z
                );

                // Try to get actual force from friction constraint data
                const key = `${collision.body_a_id}_${collision.body_b_id}`;
                const normalLambda = lambdaMap.get(key);

                collision.contacts.forEach(contact => {
                    const midpoint = new THREE.Vector3(
                        (contact.point_a.x + contact.point_b.x) / 2,
                        (contact.point_a.y + contact.point_b.y) / 2,
                        (contact.point_a.z + contact.point_b.z) / 2
                    );

                    let arrowLength;
                    if (normalLambda !== undefined) {
                        // Use actual force (Newtons) with linear scale
                        const perContactLambda = normalLambda / collision.contacts.length;
                        if (perContactLambda < this.minForce) return;
                        arrowLength = perContactLambda * this.constraintScale;
                    } else {
                        // Fallback: use penetration depth as proxy
                        const magnitude = Math.abs(contact.depth);
                        if (magnitude < 0.01) return;
                        arrowLength = Math.log1p(magnitude * 100) * 0.5;
                    }

                    this.addArrow(midpoint, normal.clone(), arrowLength, 0x00ff00, this.constraintArrows);
                });
            });
        }

        // Friction force arrows (ORANGE)
        if (this.frictionEnabled && frameData.friction_constraints) {
            // Build body position lookup
            const bodyPositions = new Map();
            if (frameData.states) {
                frameData.states.forEach(s => {
                    bodyPositions.set(s.body_id, s.position);
                });
            }

            frameData.friction_constraints.forEach(fc => {
                // Compute contact point from body A position + lever_arm_a
                const posA = bodyPositions.get(fc.body_a_id);
                if (!posA) return;

                const contactPoint = new THREE.Vector3(
                    posA.x + fc.lever_arm_a.x,
                    posA.y + fc.lever_arm_a.y,
                    posA.z + fc.lever_arm_a.z
                );

                // Use actual solved tangent lambdas for arrow magnitude
                const t1Lambda = Math.abs(fc.tangent1_lambda || 0);
                const t2Lambda = Math.abs(fc.tangent2_lambda || 0);

                const t1 = new THREE.Vector3(fc.tangent1.x, fc.tangent1.y, fc.tangent1.z);
                const t2 = new THREE.Vector3(fc.tangent2.x, fc.tangent2.y, fc.tangent2.z);

                // Tangent1 arrow (show direction of actual friction force)
                if (t1Lambda >= this.minForce) {
                    const dir1 = fc.tangent1_lambda >= 0 ? t1.clone() : t1.clone().negate();
                    this.addArrow(contactPoint.clone(), dir1, t1Lambda * this.constraintScale, 0xff8c00, this.frictionArrows);
                }

                // Tangent2 arrow
                if (t2Lambda >= this.minForce) {
                    const dir2 = fc.tangent2_lambda >= 0 ? t2.clone() : t2.clone().negate();
                    this.addArrow(contactPoint.clone(), dir2, t2Lambda * this.constraintScale, 0xff8c00, this.frictionArrows);
                }
            });
        }
    }

    /**
     * Create gravity arrows for all dynamic bodies (constant, not frame-dependent)
     */
    createGravityArrows() {
        if (!this.metadata || !this.metadata.bodies) return;

        this.metadata.bodies.forEach(body => {
            // Skip environment bodies
            if (body.is_environment) return;

            // Gravity direction (downward in Z-up coordinate system)
            const direction = new THREE.Vector3(0, 0, -1);

            // Length proportional to mass
            const arrowLength = body.mass * this.gravityMagnitude * this.gravityScale;

            const arrow = new THREE.ArrowHelper(
                direction,
                new THREE.Vector3(0, 0, 0),  // Position will be updated per frame
                arrowLength,
                0x0000ff,  // Blue
                arrowLength * 0.2,
                arrowLength * 0.1
            );

            this.scene.add(arrow);
            this.gravityArrows.set(body.body_id, arrow);
        });
    }

    /**
     * Update gravity arrow positions (they follow body CoM)
     * @param {Object} frameData - Frame data with states
     */
    updateGravityPositions(frameData) {
        if (!this.gravityEnabled || !frameData || !frameData.states) return;

        frameData.states.forEach(state => {
            const arrow = this.gravityArrows.get(state.body_id);
            if (!arrow) return;

            arrow.position.set(state.position.x, state.position.y, state.position.z);
        });
    }

    /**
     * Add an arrow helper to the scene
     * @param {THREE.Vector3} origin
     * @param {THREE.Vector3} direction
     * @param {number} length
     * @param {number} color - Hex color
     * @param {Array} targetArray - Array to store the arrow for cleanup
     */
    addArrow(origin, direction, length, color, targetArray) {
        const minLength = 0.05;
        const clampedLength = Math.max(length, minLength);

        const arrow = new THREE.ArrowHelper(
            direction.normalize(),
            origin,
            clampedLength,
            color,
            clampedLength * 0.2,
            clampedLength * 0.1
        );

        this.scene.add(arrow);
        targetArray.push(arrow);
    }

    /**
     * Clear all constraint force arrows
     */
    clearConstraintArrows() {
        this.constraintArrows.forEach(arrow => {
            this.scene.remove(arrow);
            arrow.dispose();
        });
        this.constraintArrows = [];
    }

    /**
     * Clear all friction force arrows
     */
    clearFrictionArrows() {
        this.frictionArrows.forEach(arrow => {
            this.scene.remove(arrow);
            arrow.dispose();
        });
        this.frictionArrows = [];
    }

    /**
     * Clear all gravity arrows
     */
    clearGravityArrows() {
        this.gravityArrows.forEach(arrow => {
            this.scene.remove(arrow);
            arrow.dispose();
        });
        this.gravityArrows.clear();
    }

    /**
     * Clean up all resources
     */
    dispose() {
        this.clearConstraintArrows();
        this.clearFrictionArrows();
        this.clearGravityArrows();
    }
}
