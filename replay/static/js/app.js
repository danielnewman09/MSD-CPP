// Ticket: 0056e_threejs_core_visualization
// Main entry point — Initialize all components

import { DataLoader } from './data.js';
import { SceneManager } from './scene.js';
import { PlaybackController } from './playback.js';
import { UIController } from './ui.js';

/**
 * Initialize the MSD Replay Viewer application
 */
function initApp() {
    // Get canvas element
    const canvas = document.getElementById('viewport');
    if (!canvas) {
        console.error('Canvas element not found');
        return;
    }

    // Initialize core components
    const dataLoader = new DataLoader();
    const sceneManager = new SceneManager(canvas);
    const playbackController = new PlaybackController(dataLoader, sceneManager);
    const uiController = new UIController(dataLoader, sceneManager, playbackController);

    // Load simulation list
    uiController.populateSimulationList();

    console.log('MSD Replay Viewer initialized');
}

// Start application when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initApp);
} else {
    initApp();
}
