package com.acme.camera.calibration;

import java.util.List;

/**
 * The full calibration problem: a bundle of observations of one shared set of geometry by a
 * single camera (constant intrinsics/distortion, one pose per observation), plus the pixel
 * dimensions of the sensor and whether distortion coefficients are to be estimated.
 *
 * @param observations       index-aligned world-point/pixel pairs, one entry per image
 * @param width              sensor width in pixels
 * @param height             sensor height in pixels
 * @param estimateDistortion when true, k1/k2/p1/p2 are free optimization variables;
 *                           when false they are fixed at zero
 */
public record CalibrationProblem(List<CalibrationObservation> observations,
                                 int width,
                                 int height,
                                 boolean estimateDistortion) {

    /** Total number of scalar reprojection residuals (two per observed point). */
    public int residualCount() {
        int n = 0;
        for (CalibrationObservation observation : observations) {
            n += 2 * observation.pointCount();
        }
        return n;
    }
}
