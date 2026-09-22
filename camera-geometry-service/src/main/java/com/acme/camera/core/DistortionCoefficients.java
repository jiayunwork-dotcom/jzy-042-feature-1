package com.acme.camera.core;

/**
 * Brown-Conrady distortion coefficients: radial (k1, k2) and tangential (p1, p2).
 * All coefficients operate on the NORMALIZED image plane (x = X/Z, y = Y/Z), never on pixels.
 */
public record DistortionCoefficients(double k1, double k2, double p1, double p2) {

    public static final DistortionCoefficients ZERO = new DistortionCoefficients(0.0, 0.0, 0.0, 0.0);
}
