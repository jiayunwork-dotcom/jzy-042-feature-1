package com.acme.camera.calibration;

import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Stage 1 of calibration: a purely linear (closed-form) coarse estimate of the intrinsics
 * and of every view's pose. It is NOT the answer — radial/tangential distortion and the
 * K/pose coupling are nonlinear, and {@link CameraCalibrator} hands this estimate to
 * {@link LevenbergMarquardt} for iterative reprojection-error refinement (stage 2).
 *
 * <p>Two linear routes are provided and selected by the scene geometry:
 * <ul>
 *   <li><b>Non-coplanar points (3D rig, ≥6 points/view):</b> each view contributes a DLT
 *       projection matrix; RQ decomposition ({@link RqDecomposition}) splits off K, and the
 *       per-view K estimates are averaged. Poses come from a K-fixed DLT.</li>
 *   <li><b>(Near-)planar points (flat calibration board):</b> Zhang's linear method — each
 *       view's world-plane→pixel homography contributes two linear constraints on the image
 *       of the absolute conic (zero skew assumed); poses come from homography decomposition.
 *       Fewer than three usable views falls back to a heuristic K (focal ≈ image diagonal),
 *       which stage 2 still refines from reprojection error.</li>
 * </ul>
 */
public final class CalibrationInitializer {

    /** Degeneracy threshold: smallest/second-smallest singular-value ratio of a DLT stack. */
    private static final double DEGENERATE_RATIO = 1e-10;

    private final boolean estimateDistortion;

    public CalibrationInitializer(boolean estimateDistortion) {
        this.estimateDistortion = estimateDistortion;
    }

    /** Coarse parameter pack plus bookkeeping needed to build the residual model. */
    public static class Initial {
        public double fx;
        public double fy;
        public double cx;
        public double cy;
        public final double[][] rodrigues;
        public final double[][] translations;
        public final boolean planar;

        Initial(int views, boolean planar) {
            this.rodrigues = new double[views][3];
            this.translations = new double[views][3];
            this.planar = planar;
        }
    }

    public Initial initialize(CalibrationProblem problem) {
        boolean planar = isPlanarScene(problem);
        Initial initial = new Initial(problem.observations().size(), planar);

        if (planar) {
            initializePlanar(problem, initial);
        } else {
            initializeNonPlanar(problem, initial);
        }
        return initial;
    }

    // ------------------------------------------------------------------
    // Scene geometry: the control points are considered planar when, after
    // the best-fit plane through their union is removed, the residual extent
    // is negligible relative to their in-plane extent.
    // ------------------------------------------------------------------

    private static boolean isPlanarScene(CalibrationProblem problem) {
        List<Point3D> all = new ArrayList<>();
        for (CalibrationObservation observation : problem.observations()) {
            all.addAll(observation.worldPoints());
        }
        double[] centroid = new double[3];
        for (Point3D p : all) {
            centroid[0] += p.x();
            centroid[1] += p.y();
            centroid[2] += p.z();
        }
        centroid[0] /= all.size();
        centroid[1] /= all.size();
        centroid[2] /= all.size();

        double[][] covariance = new double[3][3];
        for (Point3D p : all) {
            double dx = p.x() - centroid[0];
            double dy = p.y() - centroid[1];
            double dz = p.z() - centroid[2];
            double[] v = {dx, dy, dz};
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    covariance[i][j] += v[i] * v[j];
                }
            }
        }
        double[] eigenvalues = new double[3];
        DenseSolver.sortedJacobi(covariance, eigenvalues);
        double inPlane = eigenvalues[2];
        if (inPlane <= 0.0) {
            return true;
        }
        // Normal direction eigenvalue below ~1e-6 of the in-plane one: a flat board.
        return eigenvalues[0] <= 1e-6 * inPlane;
    }

    // ------------------------------------------------------------------
    // Non-planar route: per-view DLT camera matrices + RQ split.
    // ------------------------------------------------------------------

    private void initializeNonPlanar(CalibrationProblem problem, Initial initial) {
        double fxSum = 0.0, fySum = 0.0, cxSum = 0.0, cySum = 0.0;
        int intrinsicsViews = 0;

        for (int view = 0; view < problem.observations().size(); view++) {
            CalibrationObservation observation = problem.observations().get(view);
            double[][] camera = dltCameraMatrix(observation, true);
            if (camera != null) {
                double[][] k = new double[3][3];
                double[][] r = new double[3][3];
                double[] t = new double[3];
                if (splitCameraMatrix(camera, k, r, t)
                        && k[0][0] > 0.0 && k[1][1] > 0.0 && isRotationMatrix(r)) {
                    fxSum += k[0][0];
                    fySum += k[1][1];
                    cxSum += k[0][2];
                    cySum += k[1][2];
                    intrinsicsViews++;
                }
            }
        }

        if (intrinsicsViews > 0) {
            initial.fx = fxSum / intrinsicsViews;
            initial.fy = fySum / intrinsicsViews;
            initial.cx = cxSum / intrinsicsViews;
            initial.cy = cySum / intrinsicsViews;
        } else {
            heuristicIntrinsics(problem, initial);
        }

        initializeAllPoses(problem, initial);
    }

    /** RQ-split a 3×4 DLT camera matrix into K, R, t with positive focal lengths. */
    static boolean splitCameraMatrix(double[][] p, double[][] kOut, double[][] rOut, double[] tOut) {
        double[][] m = {
                {p[0][0], p[0][1], p[0][2]},
                {p[1][0], p[1][1], p[1][2]},
                {p[2][0], p[2][1], p[2][2]}};
        RqDecomposition.Factors factors = RqDecomposition.decompose3x3(m);
        double[][] k = factors.r();
        double[][] q = factors.q();

        // Enforce positive diagonal of K (absorbs the RQ sign ambiguity into R).
        double[] signs = new double[3];
        for (int i = 0; i < 3; i++) {
            signs[i] = k[i][i] < 0.0 ? -1.0 : 1.0;
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                k[i][j] *= signs[i];
                q[i][j] *= signs[i];
            }
        }
        double k22 = k[2][2];
        if (k22 == 0.0) {
            return false;
        }
        // P = K·[R|t]·scale: after the sign fix the homogenous scale is α = signs[2]/k22
        // (t = α·K⁻¹·p[:,3]). Normalize K to k22 = 1 FIRST, then apply that α.
        double alpha = signs[2] / k22;
        double[] p3 = {p[0][3], p[1][3], p[2][3]};
        double[][] kNormalized = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                kNormalized[i][j] = k[i][j] / k22;
            }
        }
        double[] kt = matVec(kNormalized, p3);
        for (int i = 0; i < 3; i++) {
            tOut[i] = alpha * kt[i];
        }
        for (int i = 0; i < 3; i++) {
            System.arraycopy(q[i], 0, rOut[i], 0, 3);
            System.arraycopy(kNormalized[i], 0, kOut[i], 0, 3);
        }
        // Camera must look toward the points: det(R) = +1.
        double det = determinant3(q);
        if (det < 0.0) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    rOut[i][j] = -rOut[i][j];
                }
                tOut[i] = -tOut[i];
            }
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Planar route (Zhang, zero skew).
    // ------------------------------------------------------------------

    private void initializePlanar(CalibrationProblem problem, Initial initial) {
        // Per-view world-plane bases (shared point layout: use each view's own world points).
        List<PlaneBasis> bases = new ArrayList<>();
        List<double[][]> homographies = new ArrayList<>();
        for (CalibrationObservation observation : problem.observations()) {
            PlaneBasis basis = PlaneBasis.fit(observation.worldPoints());
            double[][] h = planarHomography(observation, basis);
            if (h == null) {
                homographies.add(null);
            } else {
                homographies.add(h);
            }
            bases.add(basis);
        }

        if (!zhangIntrinsics(homographies, initial)) {
            heuristicIntrinsics(problem, initial);
        }
        for (int view = 0; view < problem.observations().size(); view++) {
            double[][] h = homographies.get(view);
            double[][] r;
            double[] t;
            if (h != null) {
                PoseFromHomography pose = poseFromHomography(h, initial, bases.get(view));
                r = pose.rotation;
                t = pose.translation;
            } else {
                double[][] camera = dltCameraMatrix(problem.observations().get(view), false);
                double[][] k = new double[3][3];
                r = new double[3][3];
                t = new double[3];
                if (camera == null || !splitCameraMatrix(camera, k, r, t)) {
                    r = identity();
                    t = new double[]{0.0, 0.0, distanceGuess(problem.observations().get(view), initial)};
                }
            }
            storePose(initial, view, r, t);
        }
    }

    /** Zhang's two linear constraints per homography on ω = K⁻ᵀK⁻¹ (skew = 0). */
    private boolean zhangIntrinsics(List<double[][]> homographies, Initial initial) {
        // Unknown b = [b00, b01, b11, b02, b12, b22] is six entries; zero skew removes b01,
        // leaving b = [b00, b11, b02, b12, b22] with
        // ω = [b00  0   b02; 0  b11  b12; b02 b12 b22].
        List<double[]> rows = new ArrayList<>();
        for (double[][] h : homographies) {
            if (h == null) {
                continue;
            }
            // h1ᵀωh2 = 0 and h1ᵀωh1 − h2ᵀωh2 = 0.
            double[] h1 = {h[0][0], h[1][0], h[2][0]};
            double[] h2 = {h[0][1], h[1][1], h[2][1]};
            rows.add(zhangRow(h1, h2));
            double[] r11 = zhangRow(h1, h1);
            double[] r22 = zhangRow(h2, h2);
            double[] diff = new double[5];
            for (int i = 0; i < 5; i++) {
                diff[i] = r11[i] - r22[i];
            }
            rows.add(diff);
        }
        if (rows.size() < 5) {
            return false;
        }
        double[][] ata = new double[5][5];
        for (double[] row : rows) {
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    ata[i][j] += row[i] * row[j];
                }
            }
        }
        double[] eigenvalues = new double[5];
        double[][] vectors = DenseSolver.sortedJacobi(ata, eigenvalues);
        double[] b = new double[5];
        for (int i = 0; i < 5; i++) {
            b[i] = vectors[i][0];
        }
        double b00 = b[0], b11 = b[1], b02 = b[2], b12 = b[3], b22 = b[4];
        if (!(b00 > 0.0 && b11 > 0.0)) {
            return false;
        }
        double v0 = -b12 / b11;
        double lambda = b22 - b02 * b02 / b00 - b12 * b12 / b11;
        if (!Double.isFinite(lambda) || lambda <= 0.0) {
            return false;
        }
        double alpha = Math.sqrt(lambda / b00);
        double beta = Math.sqrt(lambda / b11);
        double u0 = -b02 / b00;
        if (!Double.isFinite(alpha) || !Double.isFinite(beta) || !Double.isFinite(u0)
                || alpha <= 0.0 || beta <= 0.0) {
            return false;
        }
        initial.fx = alpha;
        initial.fy = beta;
        initial.cx = u0;
        initial.cy = v0;
        return true;
    }

    /** Row extracting [aᵀωb] over b = [b00, b11, b02, b12, b22] (ω01 = 0). */
    private static double[] zhangRow(double[] a, double[] b) {
        return new double[]{
                a[0] * b[0],
                a[1] * b[1],
                a[0] * b[2] + a[2] * b[0],
                a[1] * b[2] + a[2] * b[1],
                a[2] * b[2]};
    }

    // ------------------------------------------------------------------
    // Homographies and pose decomposition for the planar route.
    // ------------------------------------------------------------------

    /** Best-fit world plane: centroid plus two orthonormal in-plane axes. */
    static final class PlaneBasis {
        final double[] origin;
        final double[][] axes; // 3x2 (columns are the two in-plane unit vectors)

        private PlaneBasis(double[] origin, double[][] axes) {
            this.origin = origin;
            this.axes = axes;
        }

        static PlaneBasis fit(List<Point3D> points) {
            double[] c = new double[3];
            for (Point3D p : points) {
                c[0] += p.x();
                c[1] += p.y();
                c[2] += p.z();
            }
            c[0] /= points.size();
            c[1] /= points.size();
            c[2] /= points.size();
            double[][] cov = new double[3][3];
            for (Point3D p : points) {
                double[] v = {p.x() - c[0], p.y() - c[1], p.z() - c[2]};
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        cov[i][j] += v[i] * v[j];
                    }
                }
            }
            double[] eig = new double[3];
            double[][] vec = DenseSolver.sortedJacobi(cov, eig);
            // Eigenvectors are ascending; in-plane axes are the two largest.
            double[] e1 = {vec[0][2], vec[1][2], vec[2][2]};
            double[] e2 = {vec[0][1], vec[1][1], vec[2][1]};
            double[][] axes = {
                    {e1[0], e2[0]},
                    {e1[1], e2[1]},
                    {e1[2], e2[2]}};
            return new PlaneBasis(c, axes);
        }

        double[] toPlane(Point3D p) {
            double dx = p.x() - origin[0];
            double dy = p.y() - origin[1];
            double dz = p.z() - origin[2];
            double[] d = {dx, dy, dz};
            return new double[]{dot3(d, column(0)), dot3(d, column(1))};
        }

        double[] column(int j) {
            return new double[]{axes[0][j], axes[1][j], axes[2][j]};
        }
    }

    /**
     * DLT homography between plane-local coordinates (x, y) and observed pixels (≥4 points).
     * The plane-local coordinates live in a well-scaled frame (metres, centred), so the
     * direct 9×9 null-vector solve is adequately conditioned.
     */
    private static double[][] planarHomography(CalibrationObservation observation, PlaneBasis basis) {
        int n = observation.pointCount();
        if (n < 4) {
            return null;
        }
        double[][] a = new double[2 * n][9];
        for (int i = 0; i < n; i++) {
            double[] xy = basis.toPlane(observation.worldPoints().get(i));
            double u = observation.pixels().get(i).x();
            double v = observation.pixels().get(i).y();
            a[2 * i] = new double[]{xy[0], xy[1], 1.0, 0.0, 0.0, 0.0, -u * xy[0], -u * xy[1], -u};
            a[2 * i + 1] = new double[]{0.0, 0.0, 0.0, xy[0], xy[1], 1.0, -v * xy[0], -v * xy[1], -v};
        }
        double[] h = smallestNullVector(a);
        if (h == null) {
            return null;
        }
        return new double[][]{
                {h[0], h[1], h[2]},
                {h[3], h[4], h[5]},
                {h[6], h[7], h[8]}};
    }

    private record PoseFromHomography(double[][] rotation, double[] translation) {
    }

    /**
     * Recover [R|t] from a world-plane→pixel homography given K. In the fitted plane's local
     * frame (points have local z = 0), {@code H = λK[r1 r2 t]} so the decomposition has
     * exactly two candidates, λ &gt; 0 and λ &lt; 0. The plane has constant camera depth
     * {@code t.z} for every point, and cheirality demands it be positive — that picks the
     * candidate without ambiguity.
     */
    private PoseFromHomography poseFromHomography(double[][] h, Initial initial, PlaneBasis basis) {
        double[][] k = intrinsicsMatrix(initial);
        double[][] invK = invertUpperTriangular3(k);
        double[] g1 = matVec(invK, column(h, 0));
        double[] g2 = matVec(invK, column(h, 1));
        double[] g3 = matVec(invK, column(h, 2));
        // H = λK[r1 r2 t], so |g1| = |g2| = 1/λ. Orthonormalize the unscaled columns first,
        // then apply λ exactly once (to all columns including t).
        double lambda = 1.0 / Math.sqrt(0.5 * (dot3(g1, g1) + dot3(g2, g2)));

        double[] r1 = normalize(g1);
        double[] r2 = normalize(minus(g2, scaleVec(r1, dot3(g2, r1))));
        double[] r3 = cross(r1, r2);
        double[] tLocal = scaleVec(g3, lambda);
        // The decomposition has two candidates, ±λ; cheirality (plane depth t.z > 0) picks one.
        if (tLocal[2] < 0.0) {
            lambda = -lambda;
            r1 = scaleVec(r1, -1.0);
            r2 = scaleVec(r2, -1.0);
            tLocal = scaleVec(g3, lambda);
        }
        double[][] rLocal = {
                {r1[0], r2[0], r3[0]},
                {r1[1], r2[1], r3[1]},
                {r1[2], r2[2], r3[2]}};

        // Local-plane → world frame: p_local = Eᵀ(X − origin).
        double[] e1 = basis.column(0);
        double[] e2 = basis.column(1);
        double[] normal = cross(e1, e2);
        double[][] e = {
                {e1[0], e2[0], normal[0]},
                {e1[1], e2[1], normal[1]},
                {e1[2], e2[2], normal[2]}};
        if (determinant3(e) < 0.0) {
            for (int i = 0; i < 3; i++) {
                e[i][2] = -e[i][2];
            }
        }
        double[][] r = multiply(rLocal, transpose(e));
        double[] originOffset = matVec(r, basis.origin);
        double[] tWorld = {
                tLocal[0] - originOffset[0],
                tLocal[1] - originOffset[1],
                tLocal[2] - originOffset[2]};
        return new PoseFromHomography(r, tWorld);
    }

    // ------------------------------------------------------------------
    // DLT camera matrix and K-fixed pose recovery.
    // ------------------------------------------------------------------

    /**
     * Full 3×4 DLT projection matrix from world points to raw pixels.
     *
     * <p>Coordinates are isotropically normalized (Hartley) before the null-vector solve and
     * the resulting matrix is de-normalized: raw pixel magnitudes (~hundreds) otherwise make
     * the normal equations so ill-conditioned that a numerical rank is spuriously lost.
     *
     * @param requireNonPlanar when true, configurations whose null space is degenerate
     *                         (coplanar points seen without enough perspective) return null
     */
    static double[][] dltCameraMatrix(CalibrationObservation observation, boolean requireNonPlanar) {
        int n = observation.pointCount();
        double[] tPixel = similarity(observation.pixels(), true);
        List<Point3D> worldPoints = observation.worldPoints();
        double[] tWorld = similarity3(worldPoints);
        double[][] invTWorld = invertSimilarity3(tWorld);

        double[][] a = new double[2 * n][12];
        for (int i = 0; i < n; i++) {
            double[] xw = applySimilarity3(tWorld, worldPoints.get(i));
            double[] up = applySimilarity2(tPixel, observation.pixels().get(i));
            double u = up[0];
            double v = up[1];
            a[2 * i] = new double[]{
                    xw[0], xw[1], xw[2], 1.0, 0.0, 0.0, 0.0, 0.0, -u * xw[0], -u * xw[1], -u * xw[2], -u};
            a[2 * i + 1] = new double[]{
                    0.0, 0.0, 0.0, 0.0, xw[0], xw[1], xw[2], 1.0, -v * xw[0], -v * xw[1], -v * xw[2], -v};
        }
        double[] h = smallestCameraNullVector(a, requireNonPlanar);
        if (h == null) {
            return null;
        }
        double[][] pNorm = new double[][]{
                {h[0], h[1], h[2], h[3]},
                {h[4], h[5], h[6], h[7]},
                {h[8], h[9], h[10], h[11]}};
        // u = Tp · Pnorm · Tw⁻¹ · X  →  P = Tp⁻¹ · Pnorm · Tw⁻¹.
        double[][] invTPixel = invertSimilarity2(tPixel);
        double[][] temp = multiply(invTPixel, pNorm);
        return multiply(temp, invTWorld);
    }

    /** Similarity T = s·[I | −c] in homogeneous 2D (mean distance scaled to √2). */
    private static double[] similarity(List<Point2D> points, boolean scaleToSqrt2) {
        double cx = 0.0, cy = 0.0;
        for (Point2D p : points) {
            cx += p.x();
            cy += p.y();
        }
        cx /= points.size();
        cy /= points.size();
        double meanDist = 0.0;
        for (Point2D p : points) {
            meanDist += Math.hypot(p.x() - cx, p.y() - cy);
        }
        meanDist /= points.size();
        double s = meanDist > 0.0 ? (scaleToSqrt2 ? Math.sqrt(2.0) : 1.0) / meanDist : 1.0;
        return new double[]{s, -s * cx, s, -s * cy};
    }

    private static double[] applySimilarity2(double[] t, Point2D p) {
        return new double[]{t[0] * p.x() + t[1], t[2] * p.y() + t[3]};
    }

    private static double[][] invertSimilarity2(double[] t) {
        // T = [[sx,0,tx],[0,sy,ty],[0,0,1]]; inverse is [[1/sx,0,-tx/sx],[0,1/sy,-ty/sy],[0,0,1]].
        double invSx = 1.0 / t[0];
        double invSy = 1.0 / t[2];
        return new double[][]{
                {invSx, 0.0, -t[1] * invSx},
                {0.0, invSy, -t[3] * invSy},
                {0.0, 0.0, 1.0}};
    }

    /** 3D analogue of {@link #similarity}: isotropic scaling about the centroid (mean radius √3). */
    private static double[] similarity3(List<Point3D> points) {
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (Point3D p : points) {
            cx += p.x();
            cy += p.y();
            cz += p.z();
        }
        cx /= points.size();
        cy /= points.size();
        cz /= points.size();
        double meanDist = 0.0;
        for (Point3D p : points) {
            meanDist += Math.sqrt(sq(p.x() - cx) + sq(p.y() - cy) + sq(p.z() - cz));
        }
        meanDist /= points.size();
        double s = meanDist > 0.0 ? Math.sqrt(3.0) / meanDist : 1.0;
        return new double[]{s, -s * cx, s, -s * cy, s, -s * cz};
    }

    private static double[] applySimilarity3(double[] t, Point3D p) {
        return new double[]{t[0] * p.x() + t[1], t[2] * p.y() + t[3], t[4] * p.z() + t[5]};
    }

    private static double[][] invertSimilarity3(double[] t) {
        // Inverse of [diag(s) | translation]: [[1/s,0,0,-tx/s], ...].
        double invSx = 1.0 / t[0];
        double invSy = 1.0 / t[2];
        double invSz = 1.0 / t[4];
        return new double[][]{
                {invSx, 0.0, 0.0, -t[1] * invSx},
                {0.0, invSy, 0.0, -t[3] * invSy},
                {0.0, 0.0, invSz, -t[5] * invSz}};
    }

    private static double sq(double x) {
        return x * x;
    }

    private static double[] smallestCameraNullVector(double[][] a, boolean requireNonPlanar) {
        double[][] ata = new double[12][12];
        for (double[] row : a) {
            for (int i = 0; i < 12; i++) {
                for (int j = 0; j < 12; j++) {
                    ata[i][j] += row[i] * row[j];
                }
            }
        }
        double[] eigenvalues = new double[12];
        double[][] vectors = DenseSolver.sortedJacobi(ata, eigenvalues);
        double largest = eigenvalues[11];
        if (largest <= 0.0) {
            return null;
        }
        // A unique pinhole camera has exactly ONE (near-)zero singular value: the smallest
        // eigenvalue negligible vs the largest, but the second-smallest well separated from it.
        boolean uniqueNull = eigenvalues[0] <= DEGENERATE_RATIO * largest
                && eigenvalues[1] > 1e-6 * largest;
        if (requireNonPlanar && !uniqueNull) {
            return null;
        }
        double[] h = new double[12];
        for (int i = 0; i < 12; i++) {
            h[i] = vectors[i][0];
        }
        return h;
    }

    /** Pose for every view with intrinsics fixed: DLT on normalized coordinates. */
    private void initializeAllPoses(CalibrationProblem problem, Initial initial) {
        double[][] k = intrinsicsMatrix(initial);
        double[][] invK = invertUpperTriangular3(k);
        for (int view = 0; view < problem.observations().size(); view++) {
            CalibrationObservation observation = problem.observations().get(view);
            CalibrationObservation normalized = normalizePixels(observation, invK);
            double[][] e = dltCameraMatrix(normalized, false);
            double[][] r = null;
            double[] t = null;
            boolean usable = false;
            if (e != null) {
                double[][] r0 = {
                        {e[0][0], e[0][1], e[0][2]},
                        {e[1][0], e[1][1], e[1][2]},
                        {e[2][0], e[2][1], e[2][2]}};
                double[] tRaw = {e[0][3], e[1][3], e[2][3]};
                double det = determinant3(r0);
                if (det != 0.0) {
                    // E = λ·[R|t] for normalized coordinates, so the scale of the DLT matrix is
                    // fixed by the rotation block: λ = cbrt(|det R0|); R = R0/λ, t = tRaw/λ.
                    double lambda = Math.cbrt(Math.abs(det));
                    if (lambda > 1e-12) {
                        double[][] scaled = scaleMatrix(r0, 1.0 / lambda);
                        double[][] candidateR = nearestRotation(scaled);
                        double[] candidateT = scaleVec(tRaw, 1.0 / lambda);
                        // The DLT sign is free: [R|t] and [−R|−t] project identically.
                        // Cheirality (every point in front of the camera) picks the physical one.
                        if (pointsInFrontCount(candidateR, candidateT, observation.worldPoints())
                                < observation.pointCount()) {
                            candidateR = scaleMatrix(candidateR, -1.0);
                            candidateT = scaleVec(candidateT, -1.0);
                        }
                        if (allFinite(candidateR) && finite(candidateT)
                                && pointsInFrontCount(candidateR, candidateT, observation.worldPoints())
                                == observation.pointCount()) {
                            r = candidateR;
                            t = candidateT;
                            usable = true;
                        }
                    }
                }
            }
            if (!usable) {
                r = identity();
                t = new double[]{0.0, 0.0, distanceGuess(observation, initial)};
            }
            storePose(initial, view, r, t);
        }
    }

    private static CalibrationObservation normalizePixels(CalibrationObservation observation, double[][] invK) {
        List<Point2D> pixels = new ArrayList<>(observation.pointCount());
        for (Point2D pixel : observation.pixels()) {
            double[] n = matVec(invK, new double[]{pixel.x(), pixel.y(), 1.0});
            pixels.add(new Point2D(n[0] / n[2], n[1] / n[2]));
        }
        return new CalibrationObservation(observation.worldPoints(), pixels);
    }

    private static double distanceGuess(CalibrationObservation observation, Initial initial) {
        double maxRadius = 0.0;
        double[] c = new double[3];
        for (Point3D p : observation.worldPoints()) {
            c[0] += p.x();
            c[1] += p.y();
            c[2] += p.z();
        }
        c[0] /= observation.pointCount();
        c[1] /= observation.pointCount();
        c[2] /= observation.pointCount();
        for (Point3D p : observation.worldPoints()) {
            maxRadius = Math.max(maxRadius, Math.hypot(p.x() - c[0], p.y() - c[1]));
        }
        double focal = Math.min(initial.fx, initial.fy);
        double guess = focal > 0.0 && maxRadius > 0.0 ? focal * 2.0 * maxRadius / Math.max(initial.fx, 1.0) : 5.0;
        return Math.max(1.0, guess);
    }

    private void heuristicIntrinsics(CalibrationProblem problem, Initial initial) {
        initial.fx = problem.width();
        initial.fy = problem.width();
        initial.cx = problem.width() / 2.0;
        initial.cy = problem.height() / 2.0;
    }

    private static void storePose(Initial initial, int view, double[][] r, double[] t) {
        double[] rod = RotationMath.matrixToRodrigues(r);
        initial.rodrigues[view] = rod;
        initial.translations[view] = t.clone();
    }

    // ------------------------------------------------------------------
    // Small vector/matrix helpers.
    // ------------------------------------------------------------------

    private static double[][] intrinsicsMatrix(Initial initial) {
        return new double[][]{
                {initial.fx, 0.0, initial.cx},
                {0.0, initial.fy, initial.cy},
                {0.0, 0.0, 1.0}};
    }

    /** Inverse of an upper-triangular 3x3 matrix (K has this form; K[2][2] = 1). */
    static double[][] invertUpperTriangular3(double[][] k) {
        double a = k[0][0], b = k[0][1], c = k[0][2];
        double d = k[1][1], e = k[1][2];
        double invA = 1.0 / a;
        double invD = 1.0 / d;
        return new double[][]{
                {invA, -b * invA * invD, (b * e - c * d) * invA * invD},
                {0.0, invD, -e * invD},
                {0.0, 0.0, 1.0}};
    }

    private static double[] smallestNullVector(double[][] a) {
        int rows = a.length;
        int cols = a[0].length;
        double[][] ata = new double[cols][cols];
        for (double[] row : a) {
            for (int i = 0; i < cols; i++) {
                for (int j = 0; j < cols; j++) {
                    ata[i][j] += row[i] * row[j];
                }
            }
        }
        double[] eigenvalues = new double[cols];
        double[][] vectors = DenseSolver.sortedJacobi(ata, eigenvalues);
        if (eigenvalues[cols - 1] <= 0.0) {
            return null;
        }
        // Unique homogeneous null direction: the smallest eigenvalue must be clearly smaller
        // than the second-smallest. Distorted observations make the smallest eigenvalue
        // non-tiny in absolute terms (distortion is the residual beyond the best homography),
        // so the discriminator is the GAP e1/e0, not either value relative to the max. A
        // collapsed configuration (coincident/collinear points) has e0 ≈ e1 and is rejected.
        double e0 = eigenvalues[0];
        if (e0 <= 1e-12 * eigenvalues[cols - 1]) {
            // Numerically exact null direction (an exact homography): accept.
            return nullVector(a, eigenvalues, vectors, cols);
        }
        double gap = eigenvalues[1] / e0;
        if (!Double.isFinite(gap) || gap < 100.0) {
            return null;
        }
        return nullVector(a, eigenvalues, vectors, cols);
    }

    private static double[] nullVector(double[][] a, double[] eigenvalues, double[][] vectors, int cols) {
        double[] h = new double[cols];
        for (int i = 0; i < cols; i++) {
            h[i] = vectors[i][0];
        }
        return h;
    }

    private static double[] matVec(double[][] m, double[] v) {
        double[] out = new double[m.length];
        for (int i = 0; i < m.length; i++) {
            double s = 0.0;
            for (int j = 0; j < v.length; j++) {
                s += m[i][j] * v[j];
            }
            out[i] = s;
        }
        return out;
    }

    private static double[] column(double[][] m, int col) {
        return new double[]{m[0][col], m[1][col], m[2][col]};
    }

    private static double dot3(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] normalize(double[] v) {
        double n = Math.sqrt(dot3(v, v));
        return scaleVec(v, 1.0 / n);
    }

    private static double[] scaleVec(double[] v, double s) {
        return new double[]{v[0] * s, v[1] * s, v[2] * s};
    }

    private static double[] minus(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double determinant3(double[][] m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    private static double[][] transpose(double[][] m) {
        double[][] t = new double[m[0].length][m.length];
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[0].length; j++) {
                t[j][i] = m[i][j];
            }
        }
        return t;
    }

    private static double[][] multiply(double[][] a, double[][] b) {
        double[][] out = new double[a.length][b[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < b[0].length; j++) {
                double s = 0.0;
                for (int k = 0; k < b.length; k++) {
                    s += a[i][k] * b[k][j];
                }
                out[i][j] = s;
            }
        }
        return out;
    }

    private static double[][] identity() {
        return new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    }

    private static double[][] scaleMatrix(double[][] m, double s) {
        double[][] out = new double[m.length][m[0].length];
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[0].length; j++) {
                out[i][j] = m[i][j] * s;
            }
        }
        return out;
    }

    private static boolean isRotationMatrix(double[][] r) {
        if (!allFinite(r)) {
            return false;
        }
        double det = determinant3(r);
        if (Math.abs(det - 1.0) > 1e-4) {
            return false;
        }
        for (int j = 0; j < 3; j++) {
            double n = Math.sqrt(r[0][j] * r[0][j] + r[1][j] * r[1][j] + r[2][j] * r[2][j]);
            if (Math.abs(n - 1.0) > 1e-4) {
                return false;
            }
        }
        return true;
    }

    private static boolean finite(double[] v) {
        for (double x : v) {
            if (!Double.isFinite(x)) {
                return false;
            }
        }
        return true;
    }

    private static boolean allFinite(double[][] m) {
        for (double[] row : m) {
            if (!finite(row)) {
                return false;
            }
        }
        return true;
    }

    /** Count control points lying strictly in front of a candidate pose (Zc &gt; 0). */
    private static int pointsInFrontCount(double[][] r, double[] t, List<Point3D> points) {
        int count = 0;
        for (Point3D p : points) {
            double zc = r[2][0] * p.x() + r[2][1] * p.y() + r[2][2] * p.z() + t[2];
            if (zc > 0.0) {
                count++;
            }
        }
        return count;
    }

    /** Nearest rotation matrix via modified Gram-Schmidt on the columns of m. */
    private static double[][] nearestRotation(double[][] m) {        double[] c0 = normalize(column(m, 0));
        double[] c1 = normalize(minus(column(m, 1), scaleVec(c0, dot3(column(m, 1), c0))));
        double[] c2 = cross(c0, c1);
        double[][] r = {
                {c0[0], c1[0], c2[0]},
                {c0[1], c1[1], c2[1]},
                {c0[2], c1[2], c2[2]}};
        if (determinant3(r) < 0.0) {
            for (int i = 0; i < 3; i++) {
                r[i][2] = -r[i][2];
            }
        }
        return r;
    }
}
