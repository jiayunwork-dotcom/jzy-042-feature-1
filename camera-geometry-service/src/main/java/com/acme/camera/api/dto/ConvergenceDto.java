package com.acme.camera.api.dto;

/**
 * Convergence diagnostics of the iterative refinement, reported verbatim.
 * {@code converged} is true ONLY when the RMS residual reached {@code rmsThresholdPixels}.
 * Settling at an above-threshold local minimum ({@code IMPROVEMENT_TOO_SMALL} /
 * {@code STEP_TOO_SMALL}), hitting the iteration cap, or diverging all report
 * {@code converged=false} with the exact {@code stopReason} — the caller can always tell a
 * calibration that hit its quality target from one that merely stopped, and inspect the
 * final residual.
 */
public record ConvergenceDto(boolean converged,
                            String stopReason,
                            int iterations,
                            double initialRmsError,
                            double finalRmsError,
                            double rmsThresholdPixels) {
}
