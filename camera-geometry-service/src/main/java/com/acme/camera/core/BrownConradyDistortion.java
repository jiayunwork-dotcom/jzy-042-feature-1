package com.acme.camera.core;

/**
 * Brown-Conrady lens distortion, applied strictly on the NORMALIZED image plane.
 *
 * <p>Given a normalized point (x, y) with r² = x² + y²:
 * <pre>
 *   radial  = 1 + k1·r² + k2·r⁴
 *   x_d     = x·radial + 2·p1·x·y        + p2·(r² + 2·x²)
 *   y_d     = y·radial + p1·(r² + 2·y²)  + 2·p2·x·y
 * </pre>
 *
 * <p>WARNING: r must be computed from normalized coordinates (x = X/Z, y = Y/Z).
 * Feeding pixel coordinates into r produces values off by orders of magnitude,
 * because pixels are scaled by fx/fy (typically hundreds) relative to the
 * normalized plane.
 */
public final class BrownConradyDistortion {

    private BrownConradyDistortion() {
    }

    /** Distort a normalized-plane point. With all-zero coefficients this is the identity. */
    public static Point2D distort(Point2D normalized, DistortionCoefficients d) {
        double x = normalized.x();
        double y = normalized.y();
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double radial = 1.0 + d.k1() * r2 + d.k2() * r4;
        double xDistorted = x * radial + 2.0 * d.p1() * x * y + d.p2() * (r2 + 2.0 * x * x);
        double yDistorted = y * radial + d.p1() * (r2 + 2.0 * y * y) + 2.0 * d.p2() * x * y;
        return new Point2D(xDistorted, yDistorted);
    }

    /**
     * Iteratively invert the distortion: recover the normalized undistorted point that
     * would map to {@code distorted}. Used before triangulation so that matched pixels
     * become ideal pinhole rays. Fixed-point iteration converges quickly for any
     * physically plausible coefficient set.
     */
    public static Point2D undistort(Point2D distorted, DistortionCoefficients d) {
        double x = distorted.x();
        double y = distorted.y();
        for (int i = 0; i < 20; i++) {
            double r2 = x * x + y * y;
            double r4 = r2 * r2;
            double radial = 1.0 + d.k1() * r2 + d.k2() * r4;
            double dx = 2.0 * d.p1() * x * y + d.p2() * (r2 + 2.0 * x * x);
            double dy = d.p1() * (r2 + 2.0 * y * y) + 2.0 * d.p2() * x * y;
            x = (distorted.x() - dx) / radial;
            y = (distorted.y() - dy) / radial;
        }
        return new Point2D(x, y);
    }
}
