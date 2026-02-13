// Ticket: 0056e_threejs_core_visualization
// DataLoader — REST API client with frame buffering

const API_BASE = '/api/v1';

export class DataLoader {
    constructor() {
        this.frameCache = new Map();  // frameId -> FrameData
        this.bufferSize = 50;  // Prefetch 50 frames ahead
        this.currentSimId = null;
        this.metadata = null;
        this.geometries = null;
    }

    /**
     * List available simulations
     * @returns {Promise<Array>} List of simulation objects with id and frame_count
     */
    async listSimulations() {
        const response = await fetch(`${API_BASE}/simulations`);
        if (!response.ok) {
            throw new Error(`Failed to list simulations: ${response.statusText}`);
        }
        return await response.json();
    }

    /**
     * Load simulation metadata and geometries
     * @param {string} simId - Simulation ID
     * @returns {Promise<void>}
     */
    async loadSimulation(simId) {
        this.currentSimId = simId;
        this.frameCache.clear();

        // Load metadata (frame count, body info)
        const metaResponse = await fetch(`${API_BASE}/simulations/${simId}/metadata`);
        if (!metaResponse.ok) {
            throw new Error(`Failed to load metadata: ${metaResponse.statusText}`);
        }
        this.metadata = await metaResponse.json();

        // Extract unique asset_ids from metadata (filter out null/undefined)
        const assetIds = [...new Set(
            this.metadata.bodies.map(b => b.asset_id).filter(id => id != null)
        )];

        // Load geometries filtered by asset_ids (or all if none specified)
        const assetParams = assetIds.length > 0
            ? '?' + assetIds.map(id => `asset_ids=${id}`).join('&')
            : '';
        const geoResponse = await fetch(`${API_BASE}/simulations/${simId}/assets${assetParams}`);
        if (!geoResponse.ok) {
            throw new Error(`Failed to load geometries: ${geoResponse.statusText}`);
        }
        this.geometries = await geoResponse.json();

        // Prefetch initial frames
        await this.ensureFramesBuffered(0);
    }

    /**
     * Get frame data by frame number (0-indexed)
     * @param {number} frameNumber - Frame number (0 to frame_count - 1)
     * @returns {Promise<Object>} Frame data with states array
     */
    async getFrame(frameNumber) {
        // Check cache first
        if (this.frameCache.has(frameNumber)) {
            return this.frameCache.get(frameNumber);
        }

        // Fetch single frame
        const response = await fetch(
            `${API_BASE}/simulations/${this.currentSimId}/frames/${frameNumber}/state`
        );
        if (!response.ok) {
            throw new Error(`Failed to load frame ${frameNumber}: ${response.statusText}`);
        }

        const frameData = await response.json();
        this.frameCache.set(frameNumber, frameData);

        // Trigger background buffering
        this.ensureFramesBuffered(frameNumber);

        return frameData;
    }

    /**
     * Ensure frames are buffered ahead of current position
     * @param {number} currentFrame - Current frame number
     * @returns {Promise<void>}
     */
    async ensureFramesBuffered(currentFrame) {
        if (!this.metadata) return;

        const startFrame = currentFrame;
        const endFrame = Math.min(
            currentFrame + this.bufferSize,
            this.metadata.total_frames - 1
        );

        // Find missing frames in range
        const missingFrames = [];
        for (let i = startFrame; i <= endFrame; i++) {
            if (!this.frameCache.has(i)) {
                missingFrames.push(i);
            }
        }

        if (missingFrames.length === 0) return;

        // Use bulk range endpoint for efficiency
        const firstMissing = missingFrames[0];
        const count = missingFrames.length;

        try {
            const response = await fetch(
                `${API_BASE}/simulations/${this.currentSimId}/frames/range?start=${firstMissing}&count=${count}`
            );

            if (!response.ok) {
                console.warn(`Failed to prefetch frames: ${response.statusText}`);
                return;
            }

            const frames = await response.json();
            frames.forEach((frame, index) => {
                this.frameCache.set(firstMissing + index, frame);
            });
        } catch (error) {
            console.warn('Prefetch error:', error);
        }
    }

    /**
     * Get simulation metadata
     * @returns {Object} Metadata with frame_count and bodies array
     */
    getMetadata() {
        return this.metadata;
    }

    /**
     * Get asset geometries
     * @returns {Array} Geometries array
     */
    getGeometries() {
        return this.geometries;
    }

    /**
     * Clear cache (for simulation switching)
     */
    clearCache() {
        this.frameCache.clear();
    }
}
