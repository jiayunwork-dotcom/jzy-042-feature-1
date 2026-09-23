package com.acme.camera.calibration;

/**
 * Outcome of one {@link LevenbergMarquardt} run: the best parameter vector found and the
 * machine-readable reason the loop stopped.
 */
public record LmOutcome(double[] parameters,
                       StopReason stopReason,
                       int iterations,
                       double finalCost,
                       double finalRms) {

    /**
     * True ONLY when the RMS residual reached the configured threshold. A stop on
     * {@code IMPROVEMENT_TOO_SMALL} / {@code STEP_TOO_SMALL} means the optimizer settled at a
     * local minimum above the threshold — the numeric fit may be fine, but the calibration
     * quality target was not met, so {@code converged} stays false and {@link #stopReason()}
     * tells the caller exactly how refinement ended.
     */
    public boolean converged() {
        return stopReason == StopReason.RESIDUAL_THRESHOLD_REACHED;
    }
}
