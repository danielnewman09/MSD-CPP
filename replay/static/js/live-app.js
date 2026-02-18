// Ticket: 0072c_live_simulation_frontend
// Live simulation WebSocket client and UI controller

import { SceneManager } from './scene.js';

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/** @type {SceneManager|null} */
let sceneManager = null;

/** @type {WebSocket|null} */
let socket = null;

/** @type {Array<Object>} Configured objects waiting to be spawned */
const spawnList = [];

/** Unique counter for display labels */
let spawnCounter = 0;

/** Current connection/simulation state */
let appState = 'disconnected'; // disconnected | connecting | simulating | complete | error

// ---------------------------------------------------------------------------
// DOM References (populated after DOMContentLoaded)
// ---------------------------------------------------------------------------

let elAssetSelect;
let elObjectTypeInputs;
let elPosX, elPosY, elPosZ;
let elMassInput, elMassGroup;
let elRestitutionInput;
let elFrictionInput;
let elBtnAddObject;
let elSpawnList;
let elSpawnCount;
let elTimestepSelect;
let elDurationInput;
let elBtnStart;
let elBtnStop;
let elBtnResetCamera;
let elLoadingOverlay;
let elLoadingMessage;
let elStatusState;
let elStatusTime;
let elStatusFrame;
let elStatusBodies;
let elStatusErrorMsg;

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

function initApp() {
    const canvas = document.getElementById('viewport');
    if (!canvas) {
        console.error('live-app: canvas#viewport not found');
        return;
    }

    sceneManager = new SceneManager(canvas);

    // Gather DOM references
    elAssetSelect       = document.getElementById('asset-select');
    elObjectTypeInputs  = document.querySelectorAll('input[name="object-type"]');
    elPosX              = document.getElementById('pos-x');
    elPosY              = document.getElementById('pos-y');
    elPosZ              = document.getElementById('pos-z');
    elMassInput         = document.getElementById('mass-input');
    elMassGroup         = document.getElementById('mass-group');
    elRestitutionInput  = document.getElementById('restitution-input');
    elFrictionInput     = document.getElementById('friction-input');
    elBtnAddObject      = document.getElementById('btn-add-object');
    elSpawnList         = document.getElementById('spawn-list');
    elSpawnCount        = document.getElementById('spawn-count');
    elTimestepSelect    = document.getElementById('timestep-select');
    elDurationInput     = document.getElementById('duration-input');
    elBtnStart          = document.getElementById('btn-start');
    elBtnStop           = document.getElementById('btn-stop');
    elBtnResetCamera    = document.getElementById('btn-reset-camera');
    elLoadingOverlay    = document.getElementById('loading-overlay');
    elLoadingMessage    = document.getElementById('loading-message');
    elStatusState       = document.getElementById('status-state');
    elStatusTime        = document.getElementById('status-time');
    elStatusFrame       = document.getElementById('status-frame');
    elStatusBodies      = document.getElementById('status-bodies');
    elStatusErrorMsg    = document.getElementById('status-error-msg');

    // Event listeners
    elObjectTypeInputs.forEach(input => input.addEventListener('change', onObjectTypeChange));
    elAssetSelect.addEventListener('change', updateAddButtonState);
    elBtnAddObject.addEventListener('click', onAddObject);
    elBtnStart.addEventListener('click', onStartSimulation);
    elBtnStop.addEventListener('click', onStopSimulation);
    elBtnResetCamera.addEventListener('click', () => sceneManager.resetCamera());

    // Load assets from REST endpoint
    loadAvailableAssets();
}

// ---------------------------------------------------------------------------
// Asset loading (REST)
// ---------------------------------------------------------------------------

async function loadAvailableAssets() {
    try {
        const response = await fetch('/api/v1/live/assets');
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        /** @type {Array<{asset_id: number, name: string}>} */
        const assets = await response.json();

        elAssetSelect.innerHTML = '';
        if (assets.length === 0) {
            elAssetSelect.innerHTML = '<option value="">No assets available</option>';
            return;
        }

        assets.forEach(asset => {
            const option = document.createElement('option');
            option.value = asset.name;
            option.textContent = asset.name;
            elAssetSelect.appendChild(option);
        });

        updateAddButtonState();
    } catch (err) {
        console.error('live-app: failed to load assets', err);
        elAssetSelect.innerHTML = '<option value="">Failed to load assets</option>';
        showError(`Could not load assets: ${err.message}`);
    }
}

// ---------------------------------------------------------------------------
// Object type change handler
// ---------------------------------------------------------------------------

function onObjectTypeChange() {
    const selectedType = getSelectedObjectType();
    // Mass input is only relevant for inertial objects
    if (selectedType === 'inertial') {
        elMassGroup.classList.remove('hidden');
    } else {
        elMassGroup.classList.add('hidden');
    }
}

function getSelectedObjectType() {
    for (const input of elObjectTypeInputs) {
        if (input.checked) return input.value;
    }
    return 'inertial';
}

function updateAddButtonState() {
    const hasAsset = elAssetSelect.value !== '';
    const canAdd = hasAsset && appState === 'disconnected';
    elBtnAddObject.disabled = !canAdd;
}

// ---------------------------------------------------------------------------
// Spawn list management
// ---------------------------------------------------------------------------

function onAddObject() {
    const assetName = elAssetSelect.value;
    if (!assetName) return;

    const objectType = getSelectedObjectType();
    const x = parseFloat(elPosX.value) || 0;
    const y = parseFloat(elPosY.value) || 0;
    const z = parseFloat(elPosZ.value) || 5;
    const mass = parseFloat(elMassInput.value) || 10.0;
    const restitution = Math.min(1, Math.max(0, parseFloat(elRestitutionInput.value) || 0.8));
    const friction = Math.max(0, parseFloat(elFrictionInput.value) || 0.5);

    spawnCounter += 1;
    const entry = {
        id: spawnCounter,
        asset_name: assetName,
        object_type: objectType,
        position: [x, y, z],
        orientation: [0.0, 0.0, 0.0],
        mass,
        restitution,
        friction,
        label: `${assetName} (${objectType[0].toUpperCase() + objectType.slice(1)}) @ [${x}, ${y}, ${z}]`,
    };

    spawnList.push(entry);
    renderSpawnList();
    updateStartButtonState();
}

function removeSpawnEntry(id) {
    const idx = spawnList.findIndex(e => e.id === id);
    if (idx !== -1) spawnList.splice(idx, 1);
    renderSpawnList();
    updateStartButtonState();
}

function renderSpawnList() {
    elSpawnCount.textContent = spawnList.length;

    if (spawnList.length === 0) {
        elSpawnList.innerHTML = '<li class="spawn-empty">No objects added yet.</li>';
        return;
    }

    elSpawnList.innerHTML = '';
    spawnList.forEach(entry => {
        const li = document.createElement('li');
        li.className = 'spawn-item';
        li.dataset.id = entry.id;

        const labelSpan = document.createElement('span');
        labelSpan.className = 'spawn-label';
        labelSpan.textContent = entry.label;

        const removeBtn = document.createElement('button');
        removeBtn.className = 'btn-remove';
        removeBtn.textContent = 'x';
        removeBtn.title = 'Remove object';
        removeBtn.addEventListener('click', () => removeSpawnEntry(entry.id));

        li.appendChild(labelSpan);
        li.appendChild(removeBtn);
        elSpawnList.appendChild(li);
    });
}

function updateStartButtonState() {
    elBtnStart.disabled = spawnList.length === 0 || appState !== 'disconnected';
}

// ---------------------------------------------------------------------------
// WebSocket simulation
// ---------------------------------------------------------------------------

function onStartSimulation() {
    if (spawnList.length === 0) return;
    if (socket) return; // already running

    const timestepMs = parseInt(elTimestepSelect.value, 10);
    const durationS = parseFloat(elDurationInput.value) || 30.0;

    setAppState('connecting');
    clearError();
    resetStatusCounters();

    // Build the objects payload (strip internal id / label fields)
    const objects = spawnList.map(entry => ({
        asset_name:  entry.asset_name,
        position:    entry.position,
        orientation: entry.orientation,
        object_type: entry.object_type,
        mass:        entry.mass,
        restitution: entry.restitution,
        friction:    entry.friction,
    }));

    const wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const wsUrl = `${wsProtocol}://${window.location.host}/api/v1/live`;

    socket = new WebSocket(wsUrl);

    socket.addEventListener('open', () => {
        // Phase 1: send configure message
        socket.send(JSON.stringify({ type: 'configure', objects }));
        showLoading('Configuring simulation...');
    });

    socket.addEventListener('message', event => {
        let msg;
        try {
            msg = JSON.parse(event.data);
        } catch {
            console.error('live-app: invalid JSON from server', event.data);
            return;
        }

        switch (msg.type) {
            case 'metadata':
                handleMetadata(msg, timestepMs, durationS);
                break;
            case 'frame':
                handleFrame(msg.data);
                break;
            case 'complete':
                handleComplete(msg);
                break;
            case 'error':
                handleServerError(msg.message);
                break;
            default:
                console.warn('live-app: unknown message type', msg.type);
        }
    });

    socket.addEventListener('error', () => {
        // The 'close' event will follow; handle cleanup there
        console.error('live-app: WebSocket error');
    });

    socket.addEventListener('close', event => {
        // If we were mid-simulation (not a clean complete/stop) show an error
        if (appState === 'simulating' || appState === 'connecting') {
            showError(`WebSocket disconnected (code ${event.code})`);
            setAppState('error');
        }
        socket = null;
        hideLoading();
        enableControls();
    });
}

/**
 * Handle the metadata message: build the 3D scene then send start.
 *
 * The WebSocket metadata format:
 *   { type: "metadata", bodies: [LiveBodyMetadata], assets: [AssetGeometry] }
 *
 * SceneManager.loadBodies() expects:
 *   metadata: { bodies: [{ body_id, asset_id, is_environment, ... }] }
 *   geometries: [{ asset_id, positions: Float32Array-compatible }]
 *
 * The field names match exactly — no adapter needed.
 *
 * @param {Object} msg - Metadata message from server
 * @param {number} timestepMs - Timestep in milliseconds
 * @param {number} durationS - Duration in seconds
 */
function handleMetadata(msg, timestepMs, durationS) {
    hideLoading();

    // Build metadata object matching SceneManager expectations
    const metadata = { bodies: msg.bodies };
    const geometries = msg.assets; // already { asset_id, positions, ... }

    sceneManager.loadBodies(metadata, geometries);
    sceneManager.resetCamera();

    // Update body count in status bar
    elStatusBodies.textContent = msg.bodies.length;

    // Phase 2: send start message
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({
            type: 'start',
            timestep_ms: timestepMs,
            duration_s: durationS,
        }));
        setAppState('simulating');
    }
}

/**
 * Handle a frame message: update 3D scene and status bar.
 *
 * @param {Object} frameData - Frame data with body states
 */
function handleFrame(frameData) {
    sceneManager.updateFrame(frameData);

    // Update status bar
    elStatusFrame.textContent = frameData.frame_id ?? 0;
    const simTime = typeof frameData.simulation_time === 'number'
        ? frameData.simulation_time.toFixed(3)
        : '0.000';
    elStatusTime.textContent = `${simTime} s`;
}

/**
 * Handle the complete message sent when simulation finishes.
 *
 * @param {Object} msg - Complete message with total_frames and elapsed_s
 */
function handleComplete(msg) {
    setAppState('complete');
    hideLoading();
    if (socket) {
        socket.close();
        socket = null;
    }
    enableControls();

    console.log(`live-app: simulation complete — ${msg.total_frames} frames in ${msg.elapsed_s}s`);
}

/**
 * Handle an error message from the server.
 *
 * @param {string} message - Error description
 */
function handleServerError(message) {
    showError(`Server error: ${message}`);
    setAppState('error');
    hideLoading();
    if (socket) {
        socket.close();
        socket = null;
    }
    enableControls();
}

function onStopSimulation() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ type: 'stop' }));
    }
    // Close from our side; server will send complete before close
    if (socket) {
        // Give server a brief moment to send complete, then force close
        setTimeout(() => {
            if (socket) {
                socket.close();
                socket = null;
            }
        }, 500);
    }
    setAppState('disconnected');
    hideLoading();
    enableControls();
}

// ---------------------------------------------------------------------------
// UI state helpers
// ---------------------------------------------------------------------------

/**
 * Set the application state and update UI accordingly.
 *
 * @param {'disconnected'|'connecting'|'simulating'|'complete'|'error'} state
 */
function setAppState(state) {
    appState = state;

    // Remove all state classes from status element
    elStatusState.className = 'status-value';

    switch (state) {
        case 'disconnected':
            elStatusState.textContent = 'Disconnected';
            elStatusState.classList.add('state-disconnected');
            elBtnStart.classList.remove('hidden');
            elBtnStop.classList.add('hidden');
            break;
        case 'connecting':
            elStatusState.textContent = 'Connecting';
            elStatusState.classList.add('state-connecting');
            elBtnStart.classList.add('hidden');
            elBtnStop.classList.remove('hidden');
            disableControls();
            break;
        case 'simulating':
            elStatusState.textContent = 'Simulating';
            elStatusState.classList.add('state-simulating');
            elBtnStart.classList.add('hidden');
            elBtnStop.classList.remove('hidden');
            break;
        case 'complete':
            elStatusState.textContent = 'Complete';
            elStatusState.classList.add('state-complete');
            elBtnStart.classList.remove('hidden');
            elBtnStop.classList.add('hidden');
            break;
        case 'error':
            elStatusState.textContent = 'Error';
            elStatusState.classList.add('state-error');
            elBtnStart.classList.remove('hidden');
            elBtnStop.classList.add('hidden');
            break;
        default:
            break;
    }

    updateAddButtonState();
    updateStartButtonState();
}

function disableControls() {
    elBtnAddObject.disabled = true;
    elBtnStart.disabled = true;
    elAssetSelect.disabled = true;
    elTimestepSelect.disabled = true;
    elDurationInput.disabled = true;
    elSpawnList.querySelectorAll('.btn-remove').forEach(btn => { btn.disabled = true; });
}

function enableControls() {
    elAssetSelect.disabled = false;
    elTimestepSelect.disabled = false;
    elDurationInput.disabled = false;
    elSpawnList.querySelectorAll('.btn-remove').forEach(btn => { btn.disabled = false; });
    updateAddButtonState();
    updateStartButtonState();
}

function showLoading(message = 'Loading...') {
    elLoadingMessage.textContent = message;
    elLoadingOverlay.classList.remove('hidden');
}

function hideLoading() {
    elLoadingOverlay.classList.add('hidden');
}

function showError(message) {
    elStatusErrorMsg.textContent = message;
    elStatusErrorMsg.classList.remove('hidden');
}

function clearError() {
    elStatusErrorMsg.textContent = '';
    elStatusErrorMsg.classList.add('hidden');
}

function resetStatusCounters() {
    elStatusTime.textContent = '0.000 s';
    elStatusFrame.textContent = '0';
    elStatusBodies.textContent = '0';
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initApp);
} else {
    initApp();
}
