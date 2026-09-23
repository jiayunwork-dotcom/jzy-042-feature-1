package com.acme.camera.calibration;

import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;

import java.util.List;

/**
 * The converged solution of one calibration problem.
 *
 * @param intrinsics       estimated intrinsics (fx, fy, cx, cy plus the requested sensor size)
 * @param distortion       estimated Brown-Conrady coefficients (all zero when not estimated)
 * @param poses            one world-to-camera pose per observation, in observation order
 * @param convergence      refinement diagnostics
 * @param maxReprojectionError largest per-point reprojection distance over all observations (pixels)
 * @param meanReprojectionError mean per-point reprojection distance over all observations (pixels)
 */
public record CalibrationResult(Intrinsics intrinsics,
                                DistortionCoefficients distortion,
                                List<Extrinsics> poses,
                                ConvergenceReport convergence,
                                double maxReprojectionError,
                                double meanReprojectionError) {
}
