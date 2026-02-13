// Ticket: 0056e_threejs_core_visualization
// PlaybackController — Timeline, play/pause/step/speed

export class PlaybackController {
    constructor(dataLoader, sceneManager) {
        this.dataLoader = dataLoader;
        this.sceneManager = sceneManager;

        this.currentFrame = 0;
        this.frameCount = 0;
        this.isPlaying = false;
        this.playbackSpeed = 1.0;
        this.lastFrameTime = 0;
        this.frameInterval = 16.67;  // ~60 FPS (in ms)

        this.animationFrameId = null;
    }

    /**
     * Initialize with simulation metadata
     * @param {Object} metadata - Simulation metadata
     */
    initialize(metadata) {
        this.frameCount = metadata.total_frames;
        this.currentFrame = 0;
        this.pause();
    }

    /**
     * Play animation
     */
    play() {
        if (this.isPlaying) return;
        this.isPlaying = true;
        this.lastFrameTime = performance.now();
        this.tick();
    }

    /**
     * Pause animation
     */
    pause() {
        this.isPlaying = false;
        if (this.animationFrameId !== null) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }
    }

    /**
     * Toggle play/pause
     */
    togglePlayPause() {
        if (this.isPlaying) {
            this.pause();
        } else {
            this.play();
        }
    }

    /**
     * Step forward one frame
     */
    stepForward() {
        this.pause();
        this.setFrame(Math.min(this.currentFrame + 1, this.frameCount - 1));
    }

    /**
     * Step backward one frame
     */
    stepBack() {
        this.pause();
        this.setFrame(Math.max(this.currentFrame - 1, 0));
    }

    /**
     * Set playback speed
     * @param {number} speed - Speed multiplier (0.25, 0.5, 1, 2, 4)
     */
    setSpeed(speed) {
        this.playbackSpeed = speed;
    }

    /**
     * Jump to specific frame
     * @param {number} frameNumber - Target frame (0 to frameCount - 1)
     */
    async setFrame(frameNumber) {
        if (frameNumber < 0 || frameNumber >= this.frameCount) return;

        this.currentFrame = frameNumber;
        const frameData = await this.dataLoader.getFrame(frameNumber);
        this.sceneManager.updateFrame(frameData);
    }

    /**
     * Animation tick (called via requestAnimationFrame)
     */
    async tick() {
        if (!this.isPlaying) return;

        const now = performance.now();
        const elapsed = now - this.lastFrameTime;

        // Advance frame if enough time has passed (adjusted by speed)
        const adjustedInterval = this.frameInterval / this.playbackSpeed;
        if (elapsed >= adjustedInterval) {
            this.currentFrame++;
            if (this.currentFrame >= this.frameCount) {
                // Loop back to start or pause at end
                this.currentFrame = 0;
                // this.pause();  // Uncomment to pause at end instead of looping
            }

            const frameData = await this.dataLoader.getFrame(this.currentFrame);
            this.sceneManager.updateFrame(frameData);

            this.lastFrameTime = now;
        }

        this.animationFrameId = requestAnimationFrame(() => this.tick());
    }

    /**
     * Get current frame number
     * @returns {number}
     */
    getCurrentFrame() {
        return this.currentFrame;
    }

    /**
     * Get total frame count
     * @returns {number}
     */
    getFrameCount() {
        return this.frameCount;
    }

    /**
     * Check if playing
     * @returns {boolean}
     */
    getIsPlaying() {
        return this.isPlaying;
    }
}
