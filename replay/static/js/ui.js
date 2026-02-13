// Ticket: 0056e_threejs_core_visualization
// UI controls binding and updates

// Ticket: 0056f_threejs_overlays
import { ContactOverlay } from './overlays/contacts.js';
import { ForceOverlay } from './overlays/forces.js';
import { EnergyOverlay } from './overlays/energy.js';
import { InspectorOverlay } from './overlays/inspector.js';
import { SolverOverlay } from './overlays/solver.js';

export class UIController {
    constructor(dataLoader, sceneManager, playbackController, overlaysRef) {
        this.dataLoader = dataLoader;
        this.sceneManager = sceneManager;
        this.playbackController = playbackController;
        this.overlaysRef = overlaysRef;  // Reference to window.overlays

        this.elements = {
            simSelect: document.getElementById('sim-select'),
            playPauseBtn: document.getElementById('btn-play-pause'),
            stepBackBtn: document.getElementById('btn-step-back'),
            stepForwardBtn: document.getElementById('btn-step-forward'),
            timelineSlider: document.getElementById('timeline-slider'),
            frameDisplay: document.getElementById('frame-display'),
            timeDisplay: document.getElementById('time-display'),
            speedBtns: document.querySelectorAll('.speed-btn'),
            resetCameraBtn: document.getElementById('btn-reset-camera'),
            loadingIndicator: document.getElementById('loading-indicator'),
            // Ticket: 0056f_threejs_overlays
            toggleContactPoints: document.getElementById('toggle-contact-points'),
            toggleContactNormals: document.getElementById('toggle-contact-normals'),
            toggleConstraintForces: document.getElementById('toggle-constraint-forces'),
            toggleGravity: document.getElementById('toggle-gravity'),
            toggleEnergy: document.getElementById('toggle-energy'),
            toggleInspector: document.getElementById('toggle-inspector'),
            toggleSolver: document.getElementById('toggle-solver'),
            energyChartCanvas: document.getElementById('energy-chart'),
            inspectorPanel: document.getElementById('inspector-panel').querySelector('.inspector-content'),
            solverStatus: document.getElementById('solver-status'),
        };

        this.bindEvents();
        this.startUpdateLoop();
    }

    /**
     * Bind UI event listeners
     */
    bindEvents() {
        // Simulation selection
        this.elements.simSelect.addEventListener('change', (e) => {
            this.loadSimulation(e.target.value);
        });

        // Playback controls
        this.elements.playPauseBtn.addEventListener('click', () => {
            this.playbackController.togglePlayPause();
        });

        this.elements.stepBackBtn.addEventListener('click', () => {
            this.playbackController.stepBack();
        });

        this.elements.stepForwardBtn.addEventListener('click', () => {
            this.playbackController.stepForward();
        });

        // Timeline scrubber
        this.elements.timelineSlider.addEventListener('input', (e) => {
            const frame = parseInt(e.target.value);
            this.playbackController.pause();
            this.playbackController.setFrame(frame);
        });

        // Speed controls
        this.elements.speedBtns.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const speed = parseFloat(e.target.dataset.speed);
                this.playbackController.setSpeed(speed);
                this.elements.speedBtns.forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
            });
        });

        // Camera reset
        this.elements.resetCameraBtn.addEventListener('click', () => {
            this.sceneManager.resetCamera();
        });

        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            switch (e.code) {
                case 'Space':
                    e.preventDefault();
                    this.playbackController.togglePlayPause();
                    break;
                case 'ArrowLeft':
                    e.preventDefault();
                    this.playbackController.stepBack();
                    break;
                case 'ArrowRight':
                    e.preventDefault();
                    this.playbackController.stepForward();
                    break;
                case 'Equal':  // + key
                    e.preventDefault();
                    this.increaseSpeed();
                    break;
                case 'Minus':  // - key
                    e.preventDefault();
                    this.decreaseSpeed();
                    break;
            }
        });

        // Ticket: 0056f_threejs_overlays
        // Overlay toggle controls
        if (this.elements.toggleContactPoints) {
            this.elements.toggleContactPoints.addEventListener('change', (e) => {
                if (this.overlaysRef.contacts) {
                    this.overlaysRef.contacts.setPointsEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleContactNormals) {
            this.elements.toggleContactNormals.addEventListener('change', (e) => {
                if (this.overlaysRef.contacts) {
                    this.overlaysRef.contacts.setNormalsEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleConstraintForces) {
            this.elements.toggleConstraintForces.addEventListener('change', (e) => {
                if (this.overlaysRef.forces) {
                    this.overlaysRef.forces.setConstraintEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleGravity) {
            this.elements.toggleGravity.addEventListener('change', (e) => {
                if (this.overlaysRef.forces) {
                    this.overlaysRef.forces.setGravityEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleEnergy) {
            this.elements.toggleEnergy.addEventListener('change', async (e) => {
                if (this.overlaysRef.energy) {
                    await this.overlaysRef.energy.setEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleInspector) {
            this.elements.toggleInspector.addEventListener('change', (e) => {
                if (this.overlaysRef.inspector) {
                    this.overlaysRef.inspector.setEnabled(e.target.checked);
                }
            });
        }

        if (this.elements.toggleSolver) {
            this.elements.toggleSolver.addEventListener('change', (e) => {
                if (this.overlaysRef.solver) {
                    this.overlaysRef.solver.setEnabled(e.target.checked);
                }
            });
        }
    }

    /**
     * Populate simulation selector
     */
    async populateSimulationList() {
        try {
            const simulations = await this.dataLoader.listSimulations();
            this.elements.simSelect.innerHTML = '';

            simulations.forEach(sim => {
                const option = document.createElement('option');
                option.value = sim.id;
                option.textContent = sim.name;
                this.elements.simSelect.appendChild(option);
            });

            // Auto-load first simulation
            if (simulations.length > 0) {
                this.loadSimulation(simulations[0].id);
            }
        } catch (error) {
            console.error('Failed to load simulations:', error);
            alert('Failed to load simulation list. Check console for details.');
        }
    }

    /**
     * Load a simulation by ID
     * @param {string} simId - Simulation ID
     */
    async loadSimulation(simId) {
        if (!simId) return;

        this.showLoading();

        try {
            await this.dataLoader.loadSimulation(simId);
            const metadata = this.dataLoader.getMetadata();
            const geometries = this.dataLoader.getGeometries();

            this.sceneManager.loadBodies(metadata, geometries);
            this.playbackController.initialize(metadata);

            // Update timeline range
            this.elements.timelineSlider.max = metadata.total_frames - 1;
            this.elements.timelineSlider.value = 0;

            // Ticket: 0056f_threejs_overlays
            // Initialize overlays with metadata
            this.initializeOverlays(metadata);

            // Load first frame
            await this.playbackController.setFrame(0);

            this.hideLoading();
        } catch (error) {
            console.error('Failed to load simulation:', error);
            alert(`Failed to load simulation: ${error.message}`);
            this.hideLoading();
        }
    }

    /**
     * Update UI elements based on playback state (called in loop)
     */
    updateUI() {
        const currentFrame = this.playbackController.getCurrentFrame();
        const frameCount = this.playbackController.getFrameCount();
        const isPlaying = this.playbackController.getIsPlaying();

        // Update play/pause button icon
        this.elements.playPauseBtn.textContent = isPlaying ? '⏸' : '▶';

        // Update timeline slider (only if not being dragged)
        if (!this.elements.timelineSlider.matches(':active')) {
            this.elements.timelineSlider.value = currentFrame;
        }

        // Update frame display
        this.elements.frameDisplay.textContent = `Frame ${currentFrame} / ${frameCount}`;

        // Update time display (assuming 10ms per frame from generate_test_recording)
        const simulationTime = (currentFrame * 10) / 1000;  // Convert to seconds
        this.elements.timeDisplay.textContent = `Time: ${simulationTime.toFixed(3)}s`;

        // Ticket: 0056f_threejs_overlays
        // Update energy chart current frame marker
        if (this.overlaysRef.energy) {
            this.overlaysRef.energy.updateCurrentFrame(currentFrame);
        }
    }

    /**
     * Start UI update loop
     */
    startUpdateLoop() {
        setInterval(() => this.updateUI(), 50);  // Update UI at 20 Hz
    }

    /**
     * Increase playback speed (cycle through speeds)
     */
    increaseSpeed() {
        const speeds = [0.25, 0.5, 1, 2, 4];
        const currentSpeed = this.playbackController.playbackSpeed;
        const currentIndex = speeds.indexOf(currentSpeed);
        const nextIndex = Math.min(currentIndex + 1, speeds.length - 1);
        const newSpeed = speeds[nextIndex];

        this.playbackController.setSpeed(newSpeed);
        this.elements.speedBtns.forEach(btn => {
            btn.classList.toggle('active', parseFloat(btn.dataset.speed) === newSpeed);
        });
    }

    /**
     * Decrease playback speed (cycle through speeds)
     */
    decreaseSpeed() {
        const speeds = [0.25, 0.5, 1, 2, 4];
        const currentSpeed = this.playbackController.playbackSpeed;
        const currentIndex = speeds.indexOf(currentSpeed);
        const nextIndex = Math.max(currentIndex - 1, 0);
        const newSpeed = speeds[nextIndex];

        this.playbackController.setSpeed(newSpeed);
        this.elements.speedBtns.forEach(btn => {
            btn.classList.toggle('active', parseFloat(btn.dataset.speed) === newSpeed);
        });
    }

    /**
     * Show loading indicator
     */
    showLoading() {
        this.elements.loadingIndicator.classList.remove('hidden');
    }

    /**
     * Hide loading indicator
     */
    hideLoading() {
        this.elements.loadingIndicator.classList.add('hidden');
    }

    /**
     * Ticket: 0056f_threejs_overlays
     * Initialize overlay modules after metadata is loaded
     * @param {Object} metadata
     */
    initializeOverlays(metadata) {
        // Contact overlay
        this.overlaysRef.contacts = new ContactOverlay(this.sceneManager.scene);

        // Force overlay
        this.overlaysRef.forces = new ForceOverlay(this.sceneManager.scene, metadata);

        // Energy overlay
        this.overlaysRef.energy = new EnergyOverlay(
            this.dataLoader,
            this.elements.energyChartCanvas
        );

        // Inspector overlay
        this.overlaysRef.inspector = new InspectorOverlay(
            this.sceneManager.camera,
            this.sceneManager.scene,
            this.sceneManager,
            metadata
        );
        this.overlaysRef.inspector.setPanelElement(
            document.getElementById('inspector-panel')
        );

        // Solver overlay
        this.overlaysRef.solver = new SolverOverlay();
        this.overlaysRef.solver.setStatusElement(this.elements.solverStatus);

        // Apply default states from checkboxes
        if (this.elements.toggleEnergy.checked) {
            this.overlaysRef.energy.setEnabled(true);
        }
    }
}
