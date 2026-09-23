package com.acme.camera.calibration;

/**
 * Diagnostics of the iterative refinement, reported verbatim to the caller so a calibration
 * can never look successful when it was not.
 *
 * @param converged          true only when an explicit stopping threshold was reached
 * @param stopReason         which stopping condition fired
 * @param iterations         number of parameter-update rounds actually executed
 * @param initialRmsError    RMS reprojection error of the linear initialization (pixels)
 * @param finalRmsError      RMS reprojection error of the converged/final parameters (pixels)
 * @param rmsThresholdPixels the RMS threshold that counts as convergence
 */
public record ConvergenceReport(boolean converged,
                                StopReason stopReason,
                                int iterations,
                                double initialRmsError,
                                double finalRmsError,
                                double rmsThresholdPixels) {
}
