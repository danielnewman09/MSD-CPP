// Ticket: 0056f_threejs_overlays
// Tabbed graph overlay (Chart.js) — Energy, Linear Velocity, Angular Velocity

export class EnergyOverlay {
    constructor(dataLoader, canvasElement) {
        this.dataLoader = dataLoader;
        this.canvas = canvasElement;
        this.chart = null;
        this.enabled = false;
        this.energyData = null;
        this.velocityData = null;
        this.currentFrame = 0;

        // Tab state
        this.activeTab = 'energy';
        this.charts = {};  // tab id -> Chart instance

        // Velocity canvases (found relative to energy canvas)
        const container = this.canvas.closest('.energy-chart');
        this.linearVelCanvas = container?.querySelector('#linear-vel-chart');
        this.angularVelCanvas = container?.querySelector('#angular-vel-chart');

        this._bindTabs(container);
    }

    _bindTabs(container) {
        if (!container) return;
        container.querySelectorAll('.graph-tab').forEach(btn => {
            btn.addEventListener('click', () => this._switchTab(btn.dataset.tab));
        });
    }

    async _switchTab(tabId) {
        this.activeTab = tabId;

        const container = this.canvas.closest('.energy-chart');
        if (!container) return;

        // Update tab button active state
        container.querySelectorAll('.graph-tab').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tab === tabId);
        });

        // Update pane visibility
        container.querySelectorAll('.graph-tab-pane').forEach(pane => {
            pane.classList.toggle('active', pane.dataset.tab === tabId);
        });

        // Lazy-load velocity data on first switch
        if ((tabId === 'linear-vel' || tabId === 'angular-vel') && !this.velocityData) {
            await this._loadVelocityData();
        }

        // Create chart for this tab if not yet created
        if (!this.charts[tabId]) {
            if (tabId === 'energy' && this.energyData) {
                this._createEnergyChart();
            } else if (tabId === 'linear-vel' && this.velocityData) {
                this._createLinearVelChart();
            } else if (tabId === 'angular-vel' && this.velocityData) {
                this._createAngularVelChart();
            }
        }
    }

    /**
     * Enable/disable graph panel
     * @param {boolean} enabled
     */
    async setEnabled(enabled) {
        this.enabled = enabled;

        if (enabled) {
            await this.loadEnergyData();
            this._createEnergyChart();
            this.show();
        } else {
            this.hide();
        }
    }

    /**
     * Load energy data from API.
     */
    async loadEnergyData() {
        if (!this.dataLoader.currentSimId) return;

        try {
            const metadata = this.dataLoader.getMetadata();
            if (!metadata || !metadata.bodies) {
                this.energyData = null;
                return;
            }

            const dynamicBodies = metadata.bodies.filter(b => !b.is_environment);
            if (dynamicBodies.length === 0) {
                this.energyData = null;
                return;
            }

            const bodyEnergyPromises = dynamicBodies.map(body =>
                fetch(`/api/v1/simulations/${this.dataLoader.currentSimId}/energy/${body.body_id}`)
                    .then(r => r.ok ? r.json() : [])
            );

            const allBodyEnergies = await Promise.all(bodyEnergyPromises);

            const frameMap = new Map();

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

            this.energyData = Array.from(frameMap.values())
                .sort((a, b) => a.frame_id - b.frame_id);

        } catch (error) {
            console.warn('Failed to load energy data:', error);
            this.energyData = null;
        }
    }

    /**
     * Load velocity data from API.
     */
    async _loadVelocityData() {
        if (!this.dataLoader.currentSimId) return;

        try {
            const metadata = this.dataLoader.getMetadata();
            if (!metadata || !metadata.bodies) return;

            const dynamicBodies = metadata.bodies.filter(b => !b.is_environment);
            if (dynamicBodies.length === 0) return;

            const promises = dynamicBodies.map(body =>
                fetch(`/api/v1/simulations/${this.dataLoader.currentSimId}/velocity/${body.body_id}`)
                    .then(r => r.ok ? r.json() : [])
            );

            const allBodyVelocities = await Promise.all(promises);

            const frameMap = new Map();

            allBodyVelocities.forEach(bodyVelocity => {
                bodyVelocity.forEach(point => {
                    const existing = frameMap.get(point.frame_id);
                    if (existing) {
                        existing.speed += point.speed;
                        existing.omega_magnitude += point.omega_magnitude;
                    } else {
                        frameMap.set(point.frame_id, {
                            frame_id: point.frame_id,
                            speed: point.speed,
                            omega_magnitude: point.omega_magnitude,
                        });
                    }
                });
            });

            // Single body: store components
            if (dynamicBodies.length === 1 && allBodyVelocities[0].length > 0) {
                allBodyVelocities[0].forEach(point => {
                    const entry = frameMap.get(point.frame_id);
                    if (entry) {
                        entry.vx = point.vx;
                        entry.vy = point.vy;
                        entry.vz = point.vz;
                        entry.omega_x = point.omega_x;
                        entry.omega_y = point.omega_y;
                        entry.omega_z = point.omega_z;
                    }
                });
            }

            this.velocityData = Array.from(frameMap.values())
                .sort((a, b) => a.frame_id - b.frame_id);

        } catch (error) {
            console.warn('Failed to load velocity data:', error);
            this.velocityData = null;
        }
    }

    // --- Chart creation ---

    _createEnergyChart() {
        if (!this.energyData || !window.Chart) return;

        if (this.charts['energy']) this.charts['energy'].destroy();

        const ctx = this.canvas.getContext('2d');
        const frames = this.energyData.map(e => e.frame_id);

        this.charts['energy'] = new Chart(ctx, {
            type: 'line',
            data: {
                labels: frames,
                datasets: [
                    {
                        label: 'Total Energy',
                        data: this.energyData.map(e => e.total_e),
                        borderColor: 'rgb(255, 255, 255)',
                        backgroundColor: 'rgba(255, 255, 255, 0.1)',
                        borderWidth: 2, pointRadius: 0, tension: 0.1
                    },
                    {
                        label: 'Linear KE',
                        data: this.energyData.map(e => e.linear_ke),
                        borderColor: 'rgb(74, 158, 255)',
                        backgroundColor: 'rgba(74, 158, 255, 0.1)',
                        borderWidth: 1.5, pointRadius: 0, tension: 0.1
                    },
                    {
                        label: 'Rotational KE',
                        data: this.energyData.map(e => e.rotational_ke),
                        borderColor: 'rgb(255, 165, 0)',
                        backgroundColor: 'rgba(255, 165, 0, 0.1)',
                        borderWidth: 1.5, pointRadius: 0, tension: 0.1
                    },
                    {
                        label: 'Potential Energy',
                        data: this.energyData.map(e => e.potential_e),
                        borderColor: 'rgb(0, 255, 127)',
                        backgroundColor: 'rgba(0, 255, 127, 0.1)',
                        borderWidth: 1.5, pointRadius: 0, tension: 0.1
                    }
                ]
            },
            options: this._chartOptions('Energy (J)')
        });

        // Keep backward compat
        this.chart = this.charts['energy'];
    }

    _createLinearVelChart() {
        if (!this.velocityData || !this.linearVelCanvas || !window.Chart) return;

        if (this.charts['linear-vel']) this.charts['linear-vel'].destroy();

        const ctx = this.linearVelCanvas.getContext('2d');
        const frames = this.velocityData.map(v => v.frame_id);
        const hasComponents = this.velocityData[0]?.vx !== undefined;

        const datasets = [
            {
                label: 'Speed |v|',
                data: this.velocityData.map(v => v.speed),
                borderColor: 'rgb(255, 255, 255)',
                backgroundColor: 'rgba(255, 255, 255, 0.1)',
                borderWidth: 2, pointRadius: 0, tension: 0.1
            },
        ];

        if (hasComponents) {
            datasets.push(
                { label: 'vx', data: this.velocityData.map(v => v.vx), borderColor: 'rgb(255, 80, 80)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
                { label: 'vy', data: this.velocityData.map(v => v.vy), borderColor: 'rgb(80, 255, 80)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
                { label: 'vz', data: this.velocityData.map(v => v.vz), borderColor: 'rgb(80, 80, 255)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
            );
        }

        this.charts['linear-vel'] = new Chart(ctx, {
            type: 'line',
            data: { labels: frames, datasets },
            options: this._chartOptions('Linear Velocity (m/s)')
        });
    }

    _createAngularVelChart() {
        if (!this.velocityData || !this.angularVelCanvas || !window.Chart) return;

        if (this.charts['angular-vel']) this.charts['angular-vel'].destroy();

        const ctx = this.angularVelCanvas.getContext('2d');
        const frames = this.velocityData.map(v => v.frame_id);
        const hasComponents = this.velocityData[0]?.omega_x !== undefined;

        const datasets = [
            {
                label: '|omega|',
                data: this.velocityData.map(v => v.omega_magnitude),
                borderColor: 'rgb(255, 255, 255)',
                backgroundColor: 'rgba(255, 255, 255, 0.1)',
                borderWidth: 2, pointRadius: 0, tension: 0.1
            },
        ];

        if (hasComponents) {
            datasets.push(
                { label: 'omega_x', data: this.velocityData.map(v => v.omega_x), borderColor: 'rgb(255, 80, 80)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
                { label: 'omega_y', data: this.velocityData.map(v => v.omega_y), borderColor: 'rgb(80, 255, 80)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
                { label: 'omega_z', data: this.velocityData.map(v => v.omega_z), borderColor: 'rgb(80, 80, 255)', borderWidth: 1, pointRadius: 0, tension: 0.1 },
            );
        }

        this.charts['angular-vel'] = new Chart(ctx, {
            type: 'line',
            data: { labels: frames, datasets },
            options: this._chartOptions('Angular Velocity (rad/s)')
        });
    }

    _chartOptions(yLabel) {
        return {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            scales: {
                x: {
                    title: { display: true, text: 'Frame', color: '#e0e0e0' },
                    ticks: { color: '#a0a0a0' },
                    grid: { color: '#3a3a3a' }
                },
                y: {
                    title: { display: true, text: yLabel, color: '#e0e0e0' },
                    ticks: { color: '#a0a0a0' },
                    grid: { color: '#3a3a3a' }
                }
            },
            plugins: {
                legend: { labels: { color: '#e0e0e0' } },
                tooltip: {
                    backgroundColor: 'rgba(42, 42, 42, 0.9)',
                    titleColor: '#e0e0e0',
                    bodyColor: '#e0e0e0',
                    borderColor: '#4a9eff',
                    borderWidth: 1
                },
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
        };
    }

    /**
     * Update current frame marker on all charts
     * @param {number} frameNumber
     */
    updateCurrentFrame(frameNumber) {
        this.currentFrame = frameNumber;

        if (!this.enabled) return;

        for (const chart of Object.values(this.charts)) {
            if (chart?.options?.plugins?.annotation) {
                chart.options.plugins.annotation.annotations.frameLine.xMin = frameNumber;
                chart.options.plugins.annotation.annotations.frameLine.xMax = frameNumber;
                chart.update('none');
            }
        }
    }

    show() {
        const container = this.canvas.closest('.energy-chart');
        if (container) container.classList.remove('hidden');
    }

    hide() {
        const container = this.canvas.closest('.energy-chart');
        if (container) container.classList.add('hidden');
    }

    dispose() {
        for (const chart of Object.values(this.charts)) {
            if (chart) chart.destroy();
        }
        this.charts = {};
        this.chart = null;
        this.velocityData = null;
    }
}
