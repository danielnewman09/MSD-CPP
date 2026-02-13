// Ticket: 0056f_threejs_overlays
// Solver diagnostics status bar

export class SolverOverlay {
    constructor() {
        this.enabled = false;
        this.statusElement = null;
    }

    /**
     * Enable/disable solver diagnostics
     * @param {boolean} enabled
     */
    setEnabled(enabled) {
        this.enabled = enabled;

        if (enabled) {
            this.show();
        } else {
            this.hide();
        }
    }

    /**
     * Update solver diagnostics from frame data
     * @param {Object} frameData
     */
    update(frameData) {
        if (!this.enabled || !this.statusElement) return;

        if (!frameData || !frameData.solver) {
            this.statusElement.innerHTML = '<em>No solver data available</em>';
            this.statusElement.classList.remove('solver-failed');
            return;
        }

        const diag = frameData.solver;

        const html = `
            <span class="solver-label">Solver:</span>
            <span class="solver-value">Iterations: ${diag.iterations}</span>
            <span class="solver-value">Residual: ${diag.residual.toExponential(2)}</span>
            <span class="solver-value">Constraints: ${diag.num_constraints}</span>
            <span class="solver-value">Contacts: ${diag.num_contacts}</span>
            <span class="solver-status ${diag.converged ? 'converged' : 'diverged'}">
                ${diag.converged ? 'CONVERGED' : 'DIVERGED'}
            </span>
        `;

        this.statusElement.innerHTML = html;

        // Highlight on non-convergence
        if (!diag.converged) {
            this.statusElement.classList.add('solver-failed');
        } else {
            this.statusElement.classList.remove('solver-failed');
        }
    }

    /**
     * Set status bar element reference
     * @param {HTMLElement} element
     */
    setStatusElement(element) {
        this.statusElement = element;
    }

    /**
     * Show status bar
     */
    show() {
        if (this.statusElement) {
            this.statusElement.classList.remove('hidden');
        }
    }

    /**
     * Hide status bar
     */
    hide() {
        if (this.statusElement) {
            this.statusElement.classList.add('hidden');
        }
    }

    /**
     * Clean up resources
     */
    dispose() {
        // No resources to clean
    }
}
