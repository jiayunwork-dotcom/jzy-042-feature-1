package com.acme.camera.api.dto;

import java.util.List;

/**
 * Calibration job request: a bundle of observations of one shared control-point geometry by
 * a single camera. The service inverts them into intrinsics (fx, fy, cx, cy), Brown–Conrady
 * coefficients (k1, k2, p1, p2) and one pose per observation by minimizing reprojection
 * error.
 *
 * @param observations       one entry per image
 * @param width              sensor width in pixels
 * @param height             sensor height in pixels
 * @param estimateDistortion when null/true, k1/k2/p1/p2 are estimated; when false they are
 *                           fixed at zero (a pinhole-only calibration)
 */
public record CalibrationJobRequest(List<CalibrationObservationDto> observations,
                                    Integer width,
                                    Integer height,
                                    Boolean estimateDistortion) {
}
