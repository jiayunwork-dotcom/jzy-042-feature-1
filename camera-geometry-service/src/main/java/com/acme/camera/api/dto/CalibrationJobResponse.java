package com.acme.camera.api.dto;

import java.util.List;

/**
 * Result of a calibration job: the estimated intrinsics and Brown–Conrady coefficients, one
 * pose per observation, convergence diagnostics, and job-level max/mean reprojection errors
 * over every observed point (pixels).
 */
public record CalibrationJobResponse(String calibrationMethod,
                                     IntrinsicsDto intrinsics,
                                     DistortionDto distortion,
                                     List<CalibratedPoseDto> poses,
                                     ConvergenceDto convergence,
                                     int observationCount,
                                     int totalPoints,
                                     Double maxReprojectionError,
                                     Double meanReprojectionError) {
}
