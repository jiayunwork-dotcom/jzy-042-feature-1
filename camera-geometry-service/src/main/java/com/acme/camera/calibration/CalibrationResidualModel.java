package com.acme.camera.calibration;

import com.acme.camera.core.BrownConradyDistortion;
import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;

/**
 * The reprojection residual model of calibration: one packed parameter vector drives the
 * exact same projection pipeline the service exposes ({@link BrownConradyDistortion} on the
 * normalized plane, then intrinsics to pixels), and each observed point contributes two
 * residuals (predicted-u − observed-u, predicted-v − observed-v).
 *
 * <p>Parameter packing ({@link #pack}) — the single source of truth for layout:
 * <pre>
 *   [0..3]   fx, fy, cx, cy
 *   [4..7]   k1, k2, p1, p2            (only when distortion is estimated; length 0 otherwise)
 *   per view i, 6 entries: rodrigues (3) then translation (3)
 * </pre>
 * A projected point at or behind the camera (Zc ≤ 0) is physically invalid for this
 * parameter trial: its residuals are reported as non-finite so the optimizer rejects the
 * trial instead of following a nonsensical gradient.
 */
public final class CalibrationResidualModel implements LmResidualModel {

    /** Indices of the global intrinsics block. */
    public static final int FX = 0;
    public static final int FY = 1;
    public static final int CX = 2;
    public static final int CY = 3;

    private static final int GLOBAL_INTRINSICS = 4;
    private static final int DISTORTION_BLOCK = 4;
    private static final int POSE_BLOCK = 6;

    private final CalibrationProblem problem;
    private final int viewCount;
    private final int parameterSize;

    public CalibrationResidualModel(CalibrationProblem problem) {
        this.problem = problem;
        this.viewCount = problem.observations().size();
        this.parameterSize = GLOBAL_INTRINSICS
                + (problem.estimateDistortion() ? DISTORTION_BLOCK : 0)
                + POSE_BLOCK * viewCount;
    }

    public int distortionOffset() {
        return GLOBAL_INTRINSICS;
    }

    public int viewCount() {
        return viewCount;
    }

    public CalibrationProblem problem() {
        return problem;
    }

    public int poseOffset(int view) {
        return GLOBAL_INTRINSICS + (problem.estimateDistortion() ? DISTORTION_BLOCK : 0)
                + POSE_BLOCK * view;
    }

    @Override
    public int parameterCount() {
        return parameterSize;
    }

    @Override
    public int residualCount() {
        return problem.residualCount();
    }

    /** Pack a full parameter state. */
    public double[] pack(double fx, double fy, double cx, double cy,
                         DistortionCoefficients distortion,
                         double[][] rodrigues, double[][] translations) {
        double[] p = new double[parameterSize];
        p[FX] = fx;
        p[FY] = fy;
        p[CX] = cx;
        p[CY] = cy;
        if (problem.estimateDistortion()) {
            int d = distortionOffset();
            p[d] = distortion.k1();
            p[d + 1] = distortion.k2();
            p[d + 2] = distortion.p1();
            p[d + 3] = distortion.p2();
        }
        for (int view = 0; view < viewCount; view++) {
            int o = poseOffset(view);
            p[o] = rodrigues[view][0];
            p[o + 1] = rodrigues[view][1];
            p[o + 2] = rodrigues[view][2];
            p[o + 3] = translations[view][0];
            p[o + 4] = translations[view][1];
            p[o + 5] = translations[view][2];
        }
        return p;
    }

    /** Unpack intrinsics from a packed vector. */
    public double fx(double[] p) {
        return p[FX];
    }

    public double fy(double[] p) {
        return p[FY];
    }

    public double cx(double[] p) {
        return p[CX];
    }

    public double cy(double[] p) {
        return p[CY];
    }

    /** Unpack the distortion block (zero coefficients when distortion is not estimated). */
    public DistortionCoefficients distortion(double[] p) {
        if (!problem.estimateDistortion()) {
            return DistortionCoefficients.ZERO;
        }
        int d = distortionOffset();
        return new DistortionCoefficients(p[d], p[d + 1], p[d + 2], p[d + 3]);
    }

    /** Unpack one view's world-to-camera pose. */
    public Extrinsics pose(double[] p, int view) {
        int o = poseOffset(view);
        double[][] r = RotationMath.rodriguesToMatrix(p[o], p[o + 1], p[o + 2]);
        double[] t = new double[]{p[o + 3], p[o + 4], p[o + 5]};
        return new Extrinsics(r, t);
    }

    @Override
    public double[] residuals(double[] parameters) {
        double fx = fx(parameters);
        double fy = fy(parameters);
        if (!Double.isFinite(fx) || !Double.isFinite(fy) || fx <= 0.0 || fy <= 0.0) {
            return invalidResiduals();
        }

        double[] residuals = new double[residualCount()];
        int row = 0;
        for (int view = 0; view < viewCount; view++) {
            CalibrationObservation observation = problem.observations().get(view);
            int count = observation.pointCount();
            for (int i = 0; i < count; i++) {
                Point2D predicted = projectWorld(parameters, view, observation.worldPoints().get(i));
                if (predicted == null) {
                    return invalidResiduals();
                }
                Point2D observed = observation.pixels().get(i);
                residuals[row++] = predicted.x() - observed.x();
                residuals[row++] = predicted.y() - observed.y();
            }
        }
        return residuals;
    }

    /** Project one world point for a parameter vector; returns null when Zc ≤ 0. */
    Point2D projectWorld(double[] parameters, int view, Point3D world) {
        Extrinsics pose = pose(parameters, view);
        Point3D camera = pose.transform(world);
        if (!Double.isFinite(camera.x()) || !Double.isFinite(camera.y()) || !Double.isFinite(camera.z())
                || camera.z() <= 0.0) {
            return null;
        }
        double xn = camera.x() / camera.z();
        double yn = camera.y() / camera.z();
        Point2D distorted = BrownConradyDistortion.distort(new Point2D(xn, yn), distortion(parameters));
        if (!Double.isFinite(distorted.x()) || !Double.isFinite(distorted.y())) {
            return null;
        }
        return new Point2D(fx(parameters) * distorted.x() + cx(parameters),
                fy(parameters) * distorted.y() + cy(parameters));
    }

    private double[] invalidResiduals() {
        double[] residuals = new double[residualCount()];
        java.util.Arrays.fill(residuals, Double.NaN);
        return residuals;
    }
}
