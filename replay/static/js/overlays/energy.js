// Ticket: 0056f_threejs_overlays
// Energy chart overlay (Chart.js)

export class EnergyOverlay {
    constructor(dataLoader, canvasElement) {
        this.dataLoader = dataLoader;
        this.canvas = canvasElement;
        this.chart = null;
        this.enabled = false;
        this.energyData = null;
        this.currentFrame = 0;
    }

    /**
     * Enable/disable energy chart
     * @param {boolean} enabled
     */
    async setEnabled(enabled) {
        this.enabled = enabled;

        if (enabled) {
            await this.loadEnergyData();
            this.createChart();
            this.show();
        } else {
            this.hide();
        }
    }

    /**
     * Load energy data from API.
     *
     * Fetches per-body energy for each dynamic body and aggregates into
     * system-level totals with KE/PE component breakdown.
     */
    async loadEnergyData() {
        if (!this.dataLoader.currentSimId) return;

        try {
            const metadata = this.dataLoader.getMetadata();
            if (!metadata || !metadata.bodies) {
                this.energyData = null;
                return;
            }

            // Get dynamic (non-environment) body IDs
            const dynamicBodies = metadata.bodies.filter(b => !b.is_environment);

            if (dynamicBodies.length === 0) {
                this.energyData = null;
                return;
            }

            // Fetch per-body energy for all dynamic bodies
            const bodyEnergyPromises = dynamicBodies.map(body =>
                fetch(`/api/v1/simulations/${this.dataLoader.currentSimId}/energy/${body.body_id}`)
                    .then(r => r.ok ? r.json() : [])
            );

            const allBodyEnergies = await Promise.all(bodyEnergyPromises);

            // Aggregate across bodies per frame_id
            const frameMap = new Map();  // frame_id -> {linear_ke, rotational_ke, potential_e, total_e}

            allBodyEnergies.forEach(bodyEnergy => {
                bodyEnergy.forEach(point => {
                    const existing = frameMap.get(point.frame_id);
                    if (existing) {
                        existing.linear_ke += point.linear_ke;
                        existing.rotational_ke += point.rotational_ke;
                        existing.potential_e += point.potential_e;
                        existing.total_e += point.total_e;
                    } else {
                        frameMap.set(point.frame_id, {
                            frame_id: point.frame_id,
                            simulation_time: point.simulation_time,
                            linear_ke: point.linear_ke,
                            rotational_ke: point.rotational_ke,
                            potential_e: point.potential_e,
                            total_e: point.total_e,
                        });
                    }
                });
            });

            // Sort by frame_id
            this.energyData = Array.from(frameMap.values())
                .sort((a, b) => a.frame_id - b.frame_id);

        } catch (error) {
            console.warn('Failed to load energy data:', error);
            this.energyData = null;
        }
    }

    /**
     * Create Chart.js chart
     */
    createChart() {
        if (!this.energyData || !window.Chart) {
            console.warn('Chart.js not loaded or no energy data available');
            return;
        }

        // Destroy existing chart
        if (this.chart) {
            this.chart.destroy();
        }

        const ctx = this.canvas.getContext('2d');

        // Extract data arrays
        const frames = this.energyData.map(e => e.frame_id);
        const totalEnergy = this.energyData.map(e => e.total_e);
        const linearKE = this.energyData.map(e => e.linear_ke);
        const rotationalKE = this.energyData.map(e => e.rotational_ke);
        const potentialEnergy = this.energyData.map(e => e.potential_e);

        this.chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: frames,
                datasets: [
                    {
                        label: 'Total Energy',
                        data: totalEnergy,
                        borderColor: 'rgb(255, 255, 255)',
                        backgroundColor: 'rgba(255, 255, 255, 0.1)',
                        borderWidth: 2,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Linear KE',
                        data: linearKE,
                        borderColor: 'rgb(74, 158, 255)',
                        backgroundColor: 'rgba(74, 158, 255, 0.1)',
                        borderWidth: 1.5,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Rotational KE',
                        data: rotationalKE,
                        borderColor: 'rgb(255, 165, 0)',
                        backgroundColor: 'rgba(255, 165, 0, 0.1)',
                        borderWidth: 1.5,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Potential Energy',
                        data: potentialEnergy,
                        borderColor: 'rgb(0, 255, 127)',
                        backgroundColor: 'rgba(0, 255, 127, 0.1)',
                        borderWidth: 1.5,
                        pointRadius: 0,
                        tension: 0.1
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false
                },
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: 'Frame',
                            color: '#e0e0e0'
                        },
                        ticks: { color: '#a0a0a0' },
                        grid: { color: '#3a3a3a' }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Energy (J)',
                            color: '#e0e0e0'
                        },
                        ticks: { color: '#a0a0a0' },
                        grid: { color: '#3a3a3a' }
                    }
                },
                plugins: {
                    legend: {
                        labels: { color: '#e0e0e0' }
                    },
                    tooltip: {
                        backgroundColor: 'rgba(42, 42, 42, 0.9)',
                        titleColor: '#e0e0e0',
                        bodyColor: '#e0e0e0',
                        borderColor: '#4a9eff',
                        borderWidth: 1
                    },
                    // Vertical marker for current frame
                    annotation: {
                        annotations: {
                            frameLine: {
                                type: 'line',
                                xMin: this.currentFrame,
                                xMax: this.currentFrame,
                                borderColor: 'rgb(255, 0, 0)',
                                borderWidth: 2,
                                label: {
                                    content: 'Current Frame',
                                    enabled: true,
                                    position: 'start',
                                    color: '#e0e0e0',
                                    backgroundColor: 'rgba(255, 0, 0, 0.8)'
                                }
                            }
                        }
                    }
                }
            }
        });
    }

    /**
     * Update current frame marker
     * @param {number} frameNumber
     */
    updateCurrentFrame(frameNumber) {
        this.currentFrame = frameNumber;

        if (!this.chart || !this.enabled) return;

        // Update annotation (if annotation plugin available)
        if (this.chart.options.plugins.annotation) {
            this.chart.options.plugins.annotation.annotations.frameLine.xMin = frameNumber;
            this.chart.options.plugins.annotation.annotations.frameLine.xMax = frameNumber;
            this.chart.update('none');  // Update without animation
        }
    }

    /**
     * Show chart container
     */
    show() {
        const container = this.canvas.parentElement;
        if (container) {
            container.classList.remove('hidden');
        }
    }

    /**
     * Hide chart container
     */
    hide() {
        const container = this.canvas.parentElement;
        if (container) {
            container.classList.add('hidden');
        }
    }

    /**
     * Clean up resources
     */
    dispose() {
        if (this.chart) {
            this.chart.destroy();
            this.chart = null;
        }
    }
}
