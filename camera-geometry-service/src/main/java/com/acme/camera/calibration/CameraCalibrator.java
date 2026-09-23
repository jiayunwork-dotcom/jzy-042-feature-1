package com.acme.camera.calibration;

import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;
import com.acme.camera.core.ProjectionResult;
import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;

import java.util.ArrayList;
import java.util.List;

/**
 * The calibration kernel: invert observations into camera parameters by minimizing
 * reprojection error — the same error every other job of this service reports.
 *
 * <p>Two stages, never one:
 * <ol>
 *   <li><b>Linear initialization</b> ({@link CalibrationInitializer}): DLT/Zhang closed-form
 *       estimate of intrinsics, distortion starts at zero, and one pose per view. This is
 *       only the starting point.</li>
 *   <li><b>Iterative refinement</b> ({@link LevenbergMarquardt} over
 *       {@link CalibrationResidualModel}): intrinsics, Brown–Conrady coefficients and all
 *       poses are jointly adjusted to minimize the sum of squared pixel reprojection
 *       residuals through the service's exact projection pipeline. The loop detects whether
 *       it is descending or diverging and reports an explicit stop reason.</li>
 * </ol>
 *
 * <p>The final max/mean reprojection errors are computed by reprojecting through
 * {@link PinholeProjector} itself — guaranteeing the returned parameters round-trip through
 * the projection kernel the service already exposes.
 */
public final class CameraCalibrator {

    public static final String METHOD = "BUNDLE_REPROJECTION_LM";

    static final int MAX_ITERATIONS = 200;
    static final double RMS_THRESHOLD_PX = 1e-9;
    static final double IMPROVEMENT_TOLERANCE = 1e-12;
    static final double STEP_TOLERANCE = 1e-10;
    static final int POSE_INIT_ITERATIONS = 50;

    private final PinholeProjector projector;

    public CameraCalibrator(PinholeProjector projector) {
        this.projector = projector;
    }

    public CalibrationResult calibrate(CalibrationProblem problem) {
        CalibrationConstraintChecker.check(problem);

        // ---- Stage 1: linear coarse estimate. ----
        CalibrationInitializer initializer = new CalibrationInitializer(problem.estimateDistortion());
        CalibrationInitializer.Initial initial = initializer.initialize(problem);

        CalibrationResidualModel model = new CalibrationResidualModel(problem);
        DistortionCoefficients initialDistortion = problem.estimateDistortion()
                ? DistortionCoefficients.ZERO : DistortionCoefficients.ZERO;
        double[] parameters = model.pack(
                initial.fx, initial.fy, initial.cx, initial.cy,
                initialDistortion, initial.rodrigues, initial.translations);

        double initialRms = rms(model, parameters);

        // On a non-planar scene the DLT gives a direct intrinsics estimate; relax the poses
        // once with intrinsics held fixed so stage 2 starts inside the trust region even with
        // sizable distortion absorbed by the linear camera matrices.
        if (!initial.planar) {
            parameters = refinePoses(model, parameters, problem.observations().size());
        }

        // ---- Stage 2: joint nonlinear least squares on reprojection error. ----
        LevenbergMarquardt lm = new LevenbergMarquardt(
                MAX_ITERATIONS, RMS_THRESHOLD_PX, IMPROVEMENT_TOLERANCE, STEP_TOLERANCE);
        LmOutcome outcome = lm.optimize(model, parameters);

        double[] finalParameters = outcome.parameters();
        if (!parametersUsable(model, finalParameters)) {
            // Refinement left the model in a physically invalid state (non-positive/non-finite
            // focal length, or a point behind a camera). No polished-but-meaningless answer is
            // returned; this is rejected like every other ill-posed job.
            throw new GeometryValidationException(ErrorType.INSUFFICIENT_CONSTRAINTS,
                    "Calibration refinement diverged into a physically invalid camera model "
                            + "(stopReason=" + outcome.stopReason()
                            + "); the observations do not constrain a usable camera. "
                            + "Add more views with varied orientation or check the correspondences.",
                    java.util.Map.of("stopReason", outcome.stopReason().name(),
                            "iterations", outcome.iterations()));
        }
        Intrinsics intrinsics = new Intrinsics(
                model.fx(finalParameters), model.fy(finalParameters),
                model.cx(finalParameters), model.cy(finalParameters),
                problem.width(), problem.height());
        DistortionCoefficients distortion = model.distortion(finalParameters);

        List<Extrinsics> poses = new ArrayList<>(problem.observations().size());
        for (int view = 0; view < problem.observations().size(); view++) {
            poses.add(model.pose(finalParameters, view));
        }

        // Final error report through the public projection kernel, not a parallel code path.
        double[] maxAndMean = reprojectionErrors(problem, intrinsics, distortion, poses);
        double finalRms = Double.isFinite(outcome.finalRms()) ? outcome.finalRms() : maxAndMean[1];

        ConvergenceReport convergence = new ConvergenceReport(
                outcome.converged(), outcome.stopReason(), outcome.iterations(),
                initialRms, finalRms, RMS_THRESHOLD_PX);

        return new CalibrationResult(intrinsics, distortion, poses, convergence,
                maxAndMean[0], maxAndMean[1]);
    }

    /** Pose-only LM passes (intrinsics/distortion frozen) used to settle the initialization. */
    private static double[] refinePoses(CalibrationResidualModel model, double[] parameters, int views) {
        boolean[] active = new boolean[model.parameterCount()];
        for (int view = 0; view < views; view++) {
            for (int k = 0; k < 6; k++) {
                active[model.poseOffset(view) + k] = true;
            }
        }
        LevenbergMarquardt poseLm = new LevenbergMarquardt(
                POSE_INIT_ITERATIONS, RMS_THRESHOLD_PX, IMPROVEMENT_TOLERANCE, STEP_TOLERANCE);
        return poseLm.optimize(model, parameters, active).parameters();
    }

    private static double rms(CalibrationResidualModel model, double[] parameters) {
        double[] r = model.residuals(parameters);
        double sum = 0.0;
        for (double v : r) {
            if (!Double.isFinite(v)) {
                return Double.POSITIVE_INFINITY;
            }
            sum += v * v;
        }
        return Math.sqrt(sum / r.length);
    }

    /** A parameter vector is usable only when every observation can be projected through it. */
    private static boolean parametersUsable(CalibrationResidualModel model, double[] parameters) {
        if (!(model.fx(parameters) > 0.0) || !(model.fy(parameters) > 0.0)
                || !Double.isFinite(model.cx(parameters)) || !Double.isFinite(model.cy(parameters))) {
            return false;
        }
        for (int view = 0; view < model.viewCount(); view++) {
            for (Point3D world : model.problem().observations().get(view).worldPoints()) {
                Point2D projected = model.projectWorld(parameters, view, world);
                if (projected == null) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Reproject every observed world point through {@link PinholeProjector} and summarize
     * per-point pixel distances. This is deliberately the public projection pipeline so the
     * reported numbers are exactly what a caller gets when they feed the result back in.
     */
    private double[] reprojectionErrors(CalibrationProblem problem,
                                        Intrinsics intrinsics,
                                        DistortionCoefficients distortion,
                                        List<Extrinsics> poses) {
        double max = 0.0;
        double sum = 0.0;
        int count = 0;
        for (int view = 0; view < problem.observations().size(); view++) {
            CalibrationObservation observation = problem.observations().get(view);
            Extrinsics pose = poses.get(view);
            for (int i = 0; i < observation.pointCount(); i++) {
                Point3D world = observation.worldPoints().get(i);
                Point3D camera = pose.transform(world);
                ProjectionResult projected = projector.project(camera, intrinsics, distortion);
                double du = projected.u() - observation.pixels().get(i).x();
                double dv = projected.v() - observation.pixels().get(i).y();
                double error = Math.hypot(du, dv);
                max = Math.max(max, error);
                sum += error;
                count++;
            }
        }
        return new double[]{max, sum / count};
    }
}
