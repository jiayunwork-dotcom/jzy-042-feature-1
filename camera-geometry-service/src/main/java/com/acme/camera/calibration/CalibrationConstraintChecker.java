package com.acme.camera.calibration;

import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;

import java.util.Map;

/**
 * Decides whether a bundle of observations carries enough independent constraints to
 * determine the calibration unknowns. Runs BEFORE any math: when the data is insufficient,
 * no polished-looking but meaningless parameter set is ever returned — the job is rejected
 * with a typed {@link ErrorType#INSUFFICIENT_CONSTRAINTS} error.
 *
 * <p>Unknowns: 4 intrinsics (+ 4 distortion when estimated) + 6 pose parameters per view.
 * Every observed point contributes 2 scalar equations. Beyond the raw equation count, the
 * geometry of the linear stage must support it: non-coplanar 3D rigs need ≥6 points in each
 * view for a unique DLT camera; planar boards need ≥4 points per view (homography).
 */
public final class CalibrationConstraintChecker {

    /** Minimum points per view to solve a planar homography linearly. */
    public static final int MIN_POINTS_PLANAR_VIEW = 4;
    /** Minimum points per view to solve a 3D DLT camera matrix linearly. */
    public static final int MIN_POINTS_NONPLANAR_VIEW = 6;
    /**
     * Minimum distinct views of a planar board. One planar view cannot separate focal length
     * from the board's depth/scale (the planar focal/depth ambiguity): a wrong intrinsics set
     * can reproject with essentially zero error, so its numbers are meaningless. Non-coplanar
     * 3D rigs are identifiable from a single view.
     */
    public static final int MIN_PLANAR_VIEWS = 2;
    /** Equation-count slack demanded over the unknown count. */
    private static final int MIN_SURPLUS = 2;

    private CalibrationConstraintChecker() {
    }

    /**
     * @throws GeometryValidationException typed {@code INSUFFICIENT_CONSTRAINTS} when the
     *                                     observations cannot determine the unknowns
     */
    public static void check(CalibrationProblem problem) {
        int views = problem.observations().size();
        if (views == 0) {
            throw new GeometryValidationException(ErrorType.EMPTY_OBSERVATION_SET,
                    "Calibration job carries zero observations; at least one view of the control points is required");
        }

        boolean planar = sceneIsPlanar(problem);
        int perViewMinimum = planar ? MIN_POINTS_PLANAR_VIEW : MIN_POINTS_NONPLANAR_VIEW;
        if (planar && views < MIN_PLANAR_VIEWS) {
            throw new GeometryValidationException(ErrorType.INSUFFICIENT_CONSTRAINTS,
                    "A planar calibration board observed in only " + views + " view cannot be "
                            + "calibrated: a single view cannot separate focal length from the "
                            + "board's depth/scale (a wrong intrinsics set can reproject perfectly). "
                            + "Provide at least " + MIN_PLANAR_VIEWS
                            + " views of the board from distinct orientations.",
                    Map.of("observationCount", views,
                            "requiredViews", MIN_PLANAR_VIEWS,
                            "geometry", "planar"));
        }
        int totalObservations = 0;
        for (int view = 0; view < views; view++) {
            int n = problem.observations().get(view).pointCount();
            totalObservations += n;
            if (n < perViewMinimum) {
                throw new GeometryValidationException(ErrorType.INSUFFICIENT_CONSTRAINTS,
                        "Observation " + view + " has only " + n + " control point(s); a "
                                + (planar ? "planar (homography)" : "non-planar (3D DLT)")
                                + " initialization requires at least " + perViewMinimum
                                + " well-distributed points per view",
                        Map.of("observation", view, "pointCount", n,
                                "requiredPerView", perViewMinimum));
            }
        }

        int unknowns = 4 + (problem.estimateDistortion() ? 4 : 0) + 6 * views;
        int equations = 2 * totalObservations;
        if (equations < unknowns + MIN_SURPLUS) {
            throw new GeometryValidationException(ErrorType.INSUFFICIENT_CONSTRAINTS,
                    "Calibration is under-constrained: " + equations + " reprojection equations for "
                            + unknowns + " unknown parameters (need at least "
                            + (unknowns + MIN_SURPLUS) + "). Add more observations or control points.",
                    Map.of("equations", equations, "unknowns", unknowns,
                            "requiredEquations", unknowns + MIN_SURPLUS,
                            "observationCount", views, "totalPoints", totalObservations));
        }
    }

    /**
     * Planarity pre-screen identical in spirit to {@link CalibrationInitializer}'s: the
     * union of all control points is planar when the variance normal to their best-fit plane
     * is negligible relative to the in-plane variance.
     */
    static boolean sceneIsPlanar(CalibrationProblem problem) {
        int total = 0;
        double[] c = new double[3];
        for (CalibrationObservation observation : problem.observations()) {
            for (var p : observation.worldPoints()) {
                c[0] += p.x();
                c[1] += p.y();
                c[2] += p.z();
                total++;
            }
        }
        c[0] /= total;
        c[1] /= total;
        c[2] /= total;
        double[][] cov = new double[3][3];
        for (CalibrationObservation observation : problem.observations()) {
            for (var p : observation.worldPoints()) {
                double[] v = {p.x() - c[0], p.y() - c[1], p.z() - c[2]};
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        cov[i][j] += v[i] * v[j];
                    }
                }
            }
        }
        double[] eigenvalues = new double[3];
        DenseSolver.sortedJacobi(cov, eigenvalues);
        return eigenvalues[2] <= 0.0 || eigenvalues[0] <= 1e-6 * eigenvalues[2];
    }
}
