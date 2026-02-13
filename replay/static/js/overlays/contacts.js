// Ticket: 0056f_threejs_overlays
// Contact point and normal visualization

import * as THREE from 'three';

export class ContactOverlay {
    constructor(scene) {
        this.scene = scene;
        this.enabled = false;
        this.showNormals = false;

        // Overlay objects
        this.contactPoints = [];  // RED spheres at contact midpoints
        this.contactNormals = [];  // RED arrows in normal direction

        // Configuration
        this.pointRadius = 0.02;
        this.minArrowLength = 0.1;  // Minimum visible length
        this.penetrationScale = 10.0;  // Length scale factor
    }

    /**
     * Enable/disable contact point visualization
     * @param {boolean} enabled
     */
    setPointsEnabled(enabled) {
        this.enabled = enabled;
        if (!enabled) {
            this.clearPoints();
        }
    }

    /**
     * Enable/disable contact normal arrows
     * @param {boolean} enabled
     */
    setNormalsEnabled(enabled) {
        this.showNormals = enabled;
        if (!enabled) {
            this.clearNormals();
        }
    }

    /**
     * Update contact visualization from frame data
     * @param {Object} frameData - Frame data with contacts array
     */
    update(frameData) {
        // Clear previous frame's overlays
        this.clearPoints();
        this.clearNormals();

        if (!frameData || !frameData.contacts) return;

        frameData.contacts.forEach(contact => {
            // Contact point (midpoint of pointA and pointB)
            if (this.enabled) {
                const midpoint = new THREE.Vector3(
                    (contact.point_a.x + contact.point_b.x) / 2,
                    (contact.point_a.y + contact.point_b.y) / 2,
                    (contact.point_a.z + contact.point_b.z) / 2
                );
                this.addContactPoint(midpoint);
            }

            // Contact normal arrow
            if (this.showNormals) {
                const midpoint = new THREE.Vector3(
                    (contact.point_a.x + contact.point_b.x) / 2,
                    (contact.point_a.y + contact.point_b.y) / 2,
                    (contact.point_a.z + contact.point_b.z) / 2
                );

                const normal = new THREE.Vector3(
                    contact.normal.x,
                    contact.normal.y,
                    contact.normal.z
                );

                // Length proportional to penetration depth (with minimum)
                const arrowLength = Math.max(
                    this.minArrowLength,
                    Math.abs(contact.penetration_depth) * this.penetrationScale
                );

                this.addContactNormal(midpoint, normal, arrowLength);
            }
        });
    }

    /**
     * Add a contact point sphere
     * @param {THREE.Vector3} position
     */
    addContactPoint(position) {
        const geometry = new THREE.SphereGeometry(this.pointRadius, 8, 8);
        const material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.copy(position);

        this.scene.add(sphere);
        this.contactPoints.push(sphere);
    }

    /**
     * Add a contact normal arrow
     * @param {THREE.Vector3} origin
     * @param {THREE.Vector3} direction
     * @param {number} length
     */
    addContactNormal(origin, direction, length) {
        const arrow = new THREE.ArrowHelper(
            direction.normalize(),
            origin,
            length,
            0xff0000,  // Red
            length * 0.2,  // Head length (20% of shaft)
            length * 0.1   // Head width (10% of shaft)
        );

        this.scene.add(arrow);
        this.contactNormals.push(arrow);
    }

    /**
     * Clear all contact point spheres
     */
    clearPoints() {
        this.contactPoints.forEach(sphere => {
            this.scene.remove(sphere);
            sphere.geometry.dispose();
            sphere.material.dispose();
        });
        this.contactPoints = [];
    }

    /**
     * Clear all contact normal arrows
     */
    clearNormals() {
        this.contactNormals.forEach(arrow => {
            this.scene.remove(arrow);
            arrow.dispose();
        });
        this.contactNormals = [];
    }

    /**
     * Clean up all resources
     */
    dispose() {
        this.clearPoints();
        this.clearNormals();
    }
}
