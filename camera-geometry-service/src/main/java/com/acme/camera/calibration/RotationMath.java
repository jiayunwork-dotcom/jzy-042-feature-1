package com.acme.camera.calibration;

/**
 * Rotation/axis-angle (Rodrigues) conversions. Poses are optimized as 3-vectors
 * (angle = |r|, axis = r/|r|) so the rotation stays unconstrained yet minimal — three
 * parameters, no orthonormality constraints needed.
 */
public final class RotationMath {

    private RotationMath() {
    }

    /** Build a 3x3 rotation matrix from a Rodrigues vector. */
    public static double[][] rodriguesToMatrix(double rx, double ry, double rz) {
        double angle = Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (angle < 1e-12) {
            // First-order expansion: R ≈ I + [r]_×.
            return new double[][]{
                    {1.0, -rz, ry},
                    {rz, 1.0, -rx},
                    {-ry, rx, 1.0}};
        }
        double x = rx / angle;
        double y = ry / angle;
        double z = rz / angle;
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        double oneMinusC = 1.0 - c;
        double xx = x * x, yy = y * y, zz = z * z;
        double xy = x * y, xz = x * z, yz = y * z;
        return new double[][]{
                {c + xx * oneMinusC, xy * oneMinusC - z * s, xz * oneMinusC + y * s},
                {xy * oneMinusC + z * s, c + yy * oneMinusC, yz * oneMinusC - x * s},
                {xz * oneMinusC - y * s, yz * oneMinusC + x * s, c + zz * oneMinusC}};
    }

    /**
     * Extract a Rodrigues vector from a (proper, det ≈ +1) rotation matrix via
     * {@code angle = arccos((tr(R) − 1) / 2)} and the skew-symmetric part.
     */
    public static double[] matrixToRodrigues(double[][] r) {
        double trace = r[0][0] + r[1][1] + r[2][2];
        double cosAngle = Math.max(-1.0, Math.min(1.0, 0.5 * (trace - 1.0)));
        double angle = Math.acos(cosAngle);
        if (angle < 1e-10) {
            return new double[]{
                    0.5 * (r[2][1] - r[1][2]),
                    0.5 * (r[0][2] - r[2][0]),
                    0.5 * (r[1][0] - r[0][1])};
        }
        double scale = angle / (2.0 * Math.sin(angle));
        return new double[]{
                scale * (r[2][1] - r[1][2]),
                scale * (r[0][2] - r[2][0]),
                scale * (r[1][0] - r[0][1])};
    }
}
