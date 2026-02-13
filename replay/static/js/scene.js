// Ticket: 0056e_threejs_core_visualization
// SceneManager — Three.js scene, bodies, rendering

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class SceneManager {
    constructor(canvas) {
        this.canvas = canvas;
        this.bodies = new Map();  // bodyId -> THREE.Mesh
        this.geometryCache = new Map();  // asset_id -> THREE.BufferGeometry

        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a1a);

        // Camera setup — Z-up coordinate system (matching MSD-CPP gravity = (0,0,-9.81))
        this.camera = new THREE.PerspectiveCamera(
            75,
            window.innerWidth / window.innerHeight,
            0.1,
            1000
        );
        this.camera.up.set(0, 0, 1);  // Z-up to match simulation
        this.defaultCameraPosition = new THREE.Vector3(10, -10, 5);
        this.defaultCameraTarget = new THREE.Vector3(0, 0, 1);
        this.resetCamera();

        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this.renderer.setSize(canvas.clientWidth, canvas.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 20, 10);
        this.scene.add(directionalLight);

        // Small axis helper at origin for orientation reference
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);

        // OrbitControls — right-click (or two-finger) to pan the focus point
        this.controls = new OrbitControls(this.camera, canvas);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.enablePan = true;
        this.controls.screenSpacePanning = true;  // Pan in screen plane (intuitive)
        this.controls.panSpeed = 1.0;
        this.controls.mouseButtons = {
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.DOLLY,
            RIGHT: THREE.MOUSE.PAN
        };

        // Window resize handler
        window.addEventListener('resize', () => this.onWindowResize());

        // Animation loop
        this.animate();
    }

    /**
     * Reset camera to default view
     */
    resetCamera() {
        this.camera.position.copy(this.defaultCameraPosition);
        this.camera.lookAt(this.defaultCameraTarget);
        if (this.controls) {
            this.controls.target.copy(this.defaultCameraTarget);
            this.controls.update();
        }
    }

    /**
     * Load bodies from metadata and geometries
     * @param {Object} metadata - Simulation metadata
     * @param {Array} geometries - Asset geometries
     */
    loadBodies(metadata, geometries) {
        // Clear existing bodies
        this.clearBodies();

        // Build geometry cache
        geometries.forEach(geo => {
            const geometry = new THREE.BufferGeometry();
            const positions = new Float32Array(geo.positions);
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geometry.computeVertexNormals();
            this.geometryCache.set(geo.asset_id, geometry);
        });

        // Create mesh for each body
        metadata.bodies.forEach(body => {
            const geometry = this.geometryCache.get(body.asset_id);
            if (!geometry) {
                console.warn(`Geometry not found for asset_id ${body.asset_id}`);
                return;
            }

            // Material: blue for dynamic, gray transparent for environment
            const material = body.is_environment
                ? new THREE.MeshStandardMaterial({
                    color: 0x808080,
                    transparent: true,
                    opacity: 0.3,
                    side: THREE.DoubleSide
                })
                : new THREE.MeshStandardMaterial({
                    color: 0x4a9eff,
                    metalness: 0.3,
                    roughness: 0.6
                });

            const mesh = new THREE.Mesh(geometry, material);
            this.scene.add(mesh);
            this.bodies.set(body.body_id, mesh);
        });
    }

    /**
     * Update body transforms from frame data
     * @param {Object} frameData - Frame data with states array
     */
    updateFrame(frameData) {
        if (!frameData || !frameData.states) return;

        frameData.states.forEach(state => {
            const mesh = this.bodies.get(state.body_id);
            if (!mesh) return;

            // Update position
            mesh.position.set(state.position.x, state.position.y, state.position.z);

            // Update quaternion
            // API returns {w, x, y, z}, Three.js Quaternion.set() takes (x, y, z, w)
            const q = state.orientation;
            mesh.quaternion.set(q.x, q.y, q.z, q.w);
        });

        // Ticket: 0056f_threejs_overlays
        // Update overlays if available
        if (window.overlays) {
            if (window.overlays.contacts) {
                window.overlays.contacts.update(frameData);
            }
            if (window.overlays.forces) {
                window.overlays.forces.update(frameData);
                window.overlays.forces.updateGravityPositions(frameData);
            }
            if (window.overlays.inspector) {
                window.overlays.inspector.updateDynamicProperties(frameData);
            }
            if (window.overlays.solver) {
                window.overlays.solver.update(frameData);
            }
        }
    }

    /**
     * Clear all bodies from scene
     */
    clearBodies() {
        this.bodies.forEach(mesh => {
            this.scene.remove(mesh);
            mesh.geometry.dispose();
            mesh.material.dispose();
        });
        this.bodies.clear();
        this.geometryCache.clear();
    }

    /**
     * Handle window resize
     */
    onWindowResize() {
        const width = this.canvas.clientWidth;
        const height = this.canvas.clientHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * Animation loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}
