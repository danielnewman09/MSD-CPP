// Ticket: 0056f_threejs_overlays
// Body selection and property inspector

import * as THREE from 'three';

export class InspectorOverlay {
    constructor(camera, scene, sceneManager, metadata) {
        this.camera = camera;
        this.scene = scene;
        this.sceneManager = sceneManager;
        this.metadata = metadata;
        this.enabled = false;

        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();

        this.selectedBody = null;
        this.selectionHighlight = null;
        this.selectionParentMesh = null;

        this.panelElement = null;

        this.setupEventListeners();
    }

    /**
     * Enable/disable inspector
     * @param {boolean} enabled
     */
    setEnabled(enabled) {
        this.enabled = enabled;

        if (!enabled) {
            this.clearSelection();
            this.hidePanel();
        } else {
            this.showPanel();
        }
    }

    /**
     * Set up click event listener
     */
    setupEventListeners() {
        const canvas = this.scene.children[0]?.parent?.domElement || document.getElementById('viewport');

        canvas.addEventListener('click', (event) => {
            if (!this.enabled) return;
            this.onCanvasClick(event);
        });
    }

    /**
     * Handle canvas click for body selection
     * @param {MouseEvent} event
     */
    onCanvasClick(event) {
        const canvas = event.target;
        const rect = canvas.getBoundingClientRect();

        // Normalize mouse coordinates (-1 to +1)
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        // Raycast
        this.raycaster.setFromCamera(this.mouse, this.camera);

        const bodies = Array.from(this.sceneManager.bodies.values());
        const intersects = this.raycaster.intersectObjects(bodies);

        if (intersects.length > 0) {
            const selectedMesh = intersects[0].object;
            const bodyId = this.getBodyIdFromMesh(selectedMesh);
            this.selectBody(bodyId);
        } else {
            this.clearSelection();
        }
    }

    /**
     * Get body ID from mesh
     * @param {THREE.Mesh} mesh
     * @returns {number|null}
     */
    getBodyIdFromMesh(mesh) {
        for (const [bodyId, bodyMesh] of this.sceneManager.bodies.entries()) {
            if (bodyMesh === mesh) return bodyId;
        }
        return null;
    }

    /**
     * Select a body
     * @param {number} bodyId
     */
    selectBody(bodyId) {
        this.clearSelection();

        this.selectedBody = bodyId;
        const mesh = this.sceneManager.bodies.get(bodyId);

        if (!mesh) return;

        // Add wireframe highlight as a CHILD of the mesh so it automatically
        // follows the body's position/rotation without manual syncing
        const wireframe = new THREE.WireframeGeometry(mesh.geometry);
        const line = new THREE.LineSegments(wireframe);
        line.material = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2 });

        mesh.add(line);
        this.selectionHighlight = line;
        this.selectionParentMesh = mesh;

        this.updatePanel(bodyId);
    }

    /**
     * Clear body selection
     */
    clearSelection() {
        if (this.selectionHighlight) {
            if (this.selectionParentMesh) {
                this.selectionParentMesh.remove(this.selectionHighlight);
            }
            this.selectionHighlight.geometry.dispose();
            this.selectionHighlight.material.dispose();
            this.selectionHighlight = null;
            this.selectionParentMesh = null;
        }

        this.selectedBody = null;
    }

    /**
     * Update inspector panel with body properties
     * @param {number} bodyId
     */
    updatePanel(bodyId) {
        if (!this.panelElement) return;

        const bodyMetadata = this.metadata.bodies.find(b => b.body_id === bodyId);
        if (!bodyMetadata) return;

        const html = `
            <div class="inspector-header">Body Inspector</div>
            <div class="inspector-row">
                <span class="label">Body ID:</span>
                <span class="value">${bodyId}</span>
            </div>
            <div class="inspector-row">
                <span class="label">Asset ID:</span>
                <span class="value">${bodyMetadata.asset_id ?? 'N/A'}</span>
            </div>
            <div class="inspector-row">
                <span class="label">Mass:</span>
                <span class="value">${bodyMetadata.mass.toFixed(2)} kg</span>
            </div>
            <div class="inspector-row">
                <span class="label">Restitution:</span>
                <span class="value">${bodyMetadata.restitution.toFixed(3)}</span>
            </div>
            <div class="inspector-row">
                <span class="label">Friction:</span>
                <span class="value">${bodyMetadata.friction.toFixed(3)}</span>
            </div>
            <div class="inspector-row">
                <span class="label">Type:</span>
                <span class="value">${bodyMetadata.is_environment ? 'Environment' : 'Dynamic'}</span>
            </div>
            <div class="inspector-section">Dynamic Properties (per frame)</div>
            <div id="inspector-dynamic">
                <em>Play simulation to see dynamic properties</em>
            </div>
        `;

        this.panelElement.innerHTML = html;
    }

    /**
     * Update dynamic properties from frame data
     * @param {Object} frameData
     */
    updateDynamicProperties(frameData) {
        if (!this.enabled || !this.selectedBody || !frameData) return;

        const state = frameData.states.find(s => s.body_id === this.selectedBody);
        if (!state) return;

        const dynamicDiv = document.getElementById('inspector-dynamic');
        if (!dynamicDiv) return;

        const vel = state.velocity;
        const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

        let html = `
            <div class="inspector-row">
                <span class="label">Position:</span>
                <span class="value">(${state.position.x.toFixed(3)}, ${state.position.y.toFixed(3)}, ${state.position.z.toFixed(3)})</span>
            </div>
            <div class="inspector-row">
                <span class="label">Velocity:</span>
                <span class="value">(${vel.x.toFixed(3)}, ${vel.y.toFixed(3)}, ${vel.z.toFixed(3)})</span>
            </div>
            <div class="inspector-row">
                <span class="label">Speed:</span>
                <span class="value">${speed.toFixed(3)} m/s</span>
            </div>
            <div class="inspector-row">
                <span class="label">Rotation:</span>
                <span class="value">${(() => {
                    const q = new THREE.Quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w);
                    const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
                    const deg = (r) => (r * 180 / Math.PI).toFixed(1);
                    return `(${deg(e.x)}, ${deg(e.y)}, ${deg(e.z)})°`;
                })()}</span>
            </div>
            ${state.angular_velocity ? `
            <div class="inspector-row">
                <span class="label">Angular Vel:</span>
                <span class="value">(${state.angular_velocity.x.toFixed(3)}, ${state.angular_velocity.y.toFixed(3)}, ${state.angular_velocity.z.toFixed(3)}) rad/s</span>
            </div>
            <div class="inspector-row">
                <span class="label">Spin Rate:</span>
                <span class="value">${Math.sqrt(
                    state.angular_velocity.x ** 2 +
                    state.angular_velocity.y ** 2 +
                    state.angular_velocity.z ** 2
                ).toFixed(3)} rad/s</span>
            </div>
            ` : ''}
        `;

        // Forces section from friction constraint data
        if (frameData.friction_constraints && frameData.friction_constraints.length > 0) {
            const bodyId = this.selectedBody;
            const relevantConstraints = frameData.friction_constraints.filter(
                fc => fc.body_a_id === bodyId || fc.body_b_id === bodyId
            );

            if (relevantConstraints.length > 0) {
                html += `<div class="inspector-section">Forces</div>`;
                relevantConstraints.forEach(fc => {
                    const otherBody = fc.body_a_id === bodyId ? fc.body_b_id : fc.body_a_id;
                    const frictionLimit = fc.friction_coefficient * fc.normal_lambda;
                    const t1 = fc.tangent1_lambda || 0;
                    const t2 = fc.tangent2_lambda || 0;
                    const tangentMag = Math.sqrt(t1 * t1 + t2 * t2);
                    html += `
                        <div class="inspector-row">
                            <span class="label">Contact w/ Body ${otherBody}:</span>
                            <span class="value"></span>
                        </div>
                        <div class="inspector-row">
                            <span class="label">&nbsp;&nbsp;Normal Force:</span>
                            <span class="value">${fc.normal_lambda.toFixed(2)} N</span>
                        </div>
                        <div class="inspector-row">
                            <span class="label">&nbsp;&nbsp;Friction Force:</span>
                            <span class="value">${tangentMag.toFixed(2)} N (t1=${t1.toFixed(2)}, t2=${t2.toFixed(2)})</span>
                        </div>
                        <div class="inspector-row">
                            <span class="label">&nbsp;&nbsp;Friction Limit:</span>
                            <span class="value">${frictionLimit.toFixed(2)} N (mu=${fc.friction_coefficient.toFixed(3)})</span>
                        </div>
                    `;
                });
            }
        }

        dynamicDiv.innerHTML = html;
    }

    /**
     * Set panel element reference
     * @param {HTMLElement} element
     */
    setPanelElement(element) {
        this.panelElement = element;
    }

    /**
     * Show inspector panel
     */
    showPanel() {
        if (this.panelElement) {
            this.panelElement.classList.remove('hidden');
        }
    }

    /**
     * Hide inspector panel
     */
    hidePanel() {
        if (this.panelElement) {
            this.panelElement.classList.add('hidden');
        }
    }

    /**
     * Clean up resources
     */
    dispose() {
        this.clearSelection();
    }
}
