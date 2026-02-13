// Ticket: 0056f_threejs_overlays
// Constraint force and gravity arrow visualization

import * as THREE from 'three';

export class ForceOverlay {
    constructor(scene, metadata) {
        this.scene = scene;
        this.metadata = metadata;
        this.constraintEnabled = false;
        this.gravityEnabled = false;

        // Overlay objects
        this.constraintArrows = [];  // GREEN arrows for constraint forces
        this.gravityArrows = new Map();  // BLUE arrows for gravity (per body, constant)

        // Configuration
        this.constraintScale = 0.5;  // Logarithmic scale factor
        this.minForce = 0.01;  // Threshold for displaying constraint forces
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
     * Update constraint force arrows from frame data
     * @param {Object} frameData - Frame data with states and forces
     */
    update(frameData) {
        // Clear previous constraint forces
        this.clearConstraintArrows();

        if (!this.constraintEnabled || !frameData) return;

        // Visualize constraint forces from collision data.
        // Each collision has a normal — show the normal as a force arrow at each
        // contact midpoint, with length proportional to penetration depth.
        if (!frameData.collisions) return;

        frameData.collisions.forEach(collision => {
            const normal = new THREE.Vector3(
                collision.normal.x,
                collision.normal.y,
                collision.normal.z
            );

            collision.contacts.forEach(contact => {
                const midpoint = new THREE.Vector3(
                    (contact.point_a.x + contact.point_b.x) / 2,
                    (contact.point_a.y + contact.point_b.y) / 2,
                    (contact.point_a.z + contact.point_b.z) / 2
                );

                // Scale arrow by penetration depth (proxy for constraint force magnitude)
                const magnitude = Math.abs(contact.depth);
                if (magnitude < this.minForce) return;

                const arrowLength = Math.log1p(magnitude * 100) * this.constraintScale;
                this.addConstraintArrow(midpoint, normal.clone(), arrowLength);
            });
        });
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
     * Add a constraint force arrow
     * @param {THREE.Vector3} origin
     * @param {THREE.Vector3} force
     * @param {number} length
     */
    addConstraintArrow(origin, force, length) {
        const arrow = new THREE.ArrowHelper(
            force.normalize(),
            origin,
            length,
            0x00ff00,  // Green
            length * 0.2,
            length * 0.1
        );

        this.scene.add(arrow);
        this.constraintArrows.push(arrow);
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
        this.clearGravityArrows();
    }
}
