package com.acme.camera.calibration;

import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Kernel-level invariants of "observations invert into camera parameters":
 * ground-truth round-trips (with and without distortion), under-constrained rejection,
 * and truthful convergence reporting.
 */
class CameraCalibratorTest {

    private final PinholeProjector projector = new PinholeProjector();
    private final CameraCalibrator calibrator = new CameraCalibrator(projector);

    private static final double PARAM_TOL = 1e-4;
    private static final double REPROJECTION_TOL = 1e-6;

    // ── Invariant 1: self-consistent loop with zero distortion ──────────────

    @Test
    void planarBoardWithZeroDistortionRecoversGroundTruthParameters() {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses());

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, false));

        assertGroundTruthIntrinsics(result);
        assertEquals(0.0, result.distortion().k1(), 0.0, "distortion fixed at zero must stay zero");
        assertEquals(0.0, result.distortion().k2(), 0.0);
        assertEquals(0.0, result.distortion().p1(), 0.0);
        assertEquals(0.0, result.distortion().p2(), 0.0);
        assertReprojectionNearZero(result);
        assertTrue(result.convergence().converged(), "zero-distortion round-trip must converge");
    }

    @Test
    void nonPlanarVolumeRecoversGroundTruthParameters() {
        List<Point3D> volume = CalibrationFixtures.volumePoints();
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                volume, CalibrationFixtures.volumePoses());

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, false));

        assertGroundTruthIntrinsics(result);
        assertReprojectionNearZero(result);
        assertTrue(result.convergence().converged());
    }

    // ── Invariant 2: non-zero distortion must be identified, not collapsed to zero ──

    @Test
    void planarBoardWithNonZeroDistortionRecoversTheCoefficients() {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, CalibrationFixtures.boardPoses());

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, true));

        assertGroundTruthIntrinsics(result);
        DistortionCoefficients d = CalibrationFixtures.DISTORTION;
        assertEquals(d.k1(), result.distortion().k1(), 1e-4, "k1 must be recovered, not collapsed to zero");
        assertEquals(d.k2(), result.distortion().k2(), 1e-4, "k2 must be recovered");
        assertEquals(d.p1(), result.distortion().p1(), 1e-5, "p1 must be recovered");
        assertEquals(d.p2(), result.distortion().p2(), 1e-5, "p2 must be recovered");
        assertReprojectionNearZero(result);
        assertTrue(result.convergence().converged());

        // And the answer must NOT be a zero-distortion calibration pretending to fit.
        assertFalse(allDistortionNearZero(result),
                "non-zero ground-truth distortion must not be solved back as zero coefficients");
    }

    @Test
    void nonPlanarVolumeWithNonZeroDistortionRecoversTheCoefficients() {
        List<Point3D> volume = CalibrationFixtures.volumePoints();
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                volume, CalibrationFixtures.volumePoses());

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, true));

        assertGroundTruthIntrinsics(result);
        assertEquals(CalibrationFixtures.DISTORTION.k1(), result.distortion().k1(), 1e-4);
        assertEquals(CalibrationFixtures.DISTORTION.k2(), result.distortion().k2(), 1e-4);
        assertEquals(CalibrationFixtures.DISTORTION.p1(), result.distortion().p1(), 1e-5);
        assertEquals(CalibrationFixtures.DISTORTION.p2(), result.distortion().p2(), 1e-5);
        assertReprojectionNearZero(result);
        assertTrue(result.convergence().converged());
    }

    // ── Invariant 3: insufficient observations are rejected with a typed error ──

    @Test
    void zeroObservationsAreRejected() {
        GeometryValidationException ex = assertThrows(GeometryValidationException.class,
                () -> calibrator.calibrate(new CalibrationProblem(List.of(), 1280, 720, true)));
        assertEquals(ErrorType.EMPTY_OBSERVATION_SET, ex.type());
    }

    @Test
    void tooFewPointsPerViewAreRejectedAsUnderConstrained() {
        List<Point3D> threePoints = List.of(
                new Point3D(0, 0, 0), new Point3D(0.3, 0, 0), new Point3D(0, 0.3, 0));
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                threePoints, List.of(CalibrationFixtures.pose(0.1, 0.1, 0, 0, 0, 3)));

        GeometryValidationException ex = assertThrows(GeometryValidationException.class,
                () -> calibrator.calibrate(new CalibrationProblem(observations, 1280, 720, false)));
        assertEquals(ErrorType.INSUFFICIENT_CONSTRAINTS, ex.type());
    }

    @Test
    void tooFewObservationsForTheUnknownCountAreRejected() {
        // Six points per planar view are plenty per-view, but a single view cannot determine
        // the shared intrinsics plus a pose (8/10 unknowns vs 12 equations is borderline; use
        // one view of four points: 8 equations, 10 unknowns with distortion).
        List<Point3D> board = CalibrationFixtures.boardPoints(2, 2, 0.3);
        List<CalibrationObservation> oneView = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, List.of(CalibrationFixtures.pose(0.1, 0.1, 0, 0, 0, 3)));

        GeometryValidationException ex = assertThrows(GeometryValidationException.class,
                () -> calibrator.calibrate(new CalibrationProblem(oneView, 1280, 720, true)));
        assertEquals(ErrorType.INSUFFICIENT_CONSTRAINTS, ex.type());
    }

    @Test
    void singlePlanarViewIsRejectedAsUnderConstrained() {
        // One planar view has the focal/depth-scale ambiguity: a wrong K can reproject perfectly,
        // so its parameters are meaningless even though the residual is zero.
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        List<CalibrationObservation> oneView = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, List.of(CalibrationFixtures.pose(0.15, -0.2, 0.05, 0.2, -0.1, 3.0)));

        GeometryValidationException ex = assertThrows(GeometryValidationException.class,
                () -> calibrator.calibrate(new CalibrationProblem(oneView, 1280, 720, true)));
        assertEquals(ErrorType.INSUFFICIENT_CONSTRAINTS, ex.type());
        assertEquals(2, ex.details().get("requiredViews"));
    }

    @Test
    void singleViewOfNonPlanarRigIsAcceptable() {
        // Unlike a flat board, a genuinely 3D rig is identifiable from one view (full P via DLT).
        List<Point3D> volume = CalibrationFixtures.volumePoints();
        List<CalibrationObservation> oneView = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                volume, List.of(CalibrationFixtures.volumePoses().get(0)));

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(oneView, 1280, 720, true));
        assertTrue(result.convergence().converged());
        assertGroundTruthIntrinsics(result);
        assertReprojectionNearZero(result);
    }

    // ── Invariant 4: convergence status is truthfully reported ──

    @Test
    void convergenceReportShowsThresholdStopAndNonZeroIterationCount() {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, CalibrationFixtures.boardPoses());

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, true));

        ConvergenceReport c = result.convergence();
        assertNotNull(c.stopReason());
        assertTrue(c.converged(), "exact synthetic data must converge");
        assertEquals(StopReason.RESIDUAL_THRESHOLD_REACHED, c.stopReason(),
                "round-trip data must stop on the residual threshold, not the iteration cap");
        assertTrue(c.iterations() > 0, "iterative refinement must actually run rounds");
        assertTrue(c.initialRmsError() > c.finalRmsError(),
                "refinement must reduce the residual from the linear initial estimate");
        assertTrue(c.finalRmsError() <= c.rmsThresholdPixels(),
                "final RMS must be at/below the reported threshold");
        assertEquals(1e-9, c.rmsThresholdPixels(), 0.0);
    }

    @Test
    void inconsistentObservationsDoNotClaimThresholdConvergence() {
        // Observations genuinely produced by TWO different cameras cannot be explained by one
        // intrinsics set. Refinement still finds the best compromise, but the response must not
        // pretend the residual threshold was reached.
        List<Point3D> board = List.of(
                new Point3D(-0.4, -0.3, 0), new Point3D(0.4, -0.3, 0),
                new Point3D(0.4, 0.3, 0), new Point3D(-0.4, 0.3, 0),
                new Point3D(0.0, -0.3, 0), new Point3D(0.4, 0.0, 0),
                new Point3D(0.0, 0.3, 0), new Point3D(-0.4, 0.0, 0));
        List<Extrinsics> poses = List.of(
                CalibrationFixtures.pose(0.10, -0.10, 0.00, 0.00, 0.00, 3.0),
                CalibrationFixtures.pose(-0.10, 0.15, 0.05, -0.20, 0.10, 3.2),
                CalibrationFixtures.pose(0.05, 0.10, -0.10, 0.10, 0.20, 2.9),
                CalibrationFixtures.pose(0.15, 0.05, 0.10, -0.10, -0.20, 3.1),
                CalibrationFixtures.pose(-0.12, -0.10, 0.05, 0.20, 0.00, 3.3),
                CalibrationFixtures.pose(0.08, -0.18, 0.00, -0.25, 0.15, 3.0));
        Intrinsics alternate = new Intrinsics(650.0, 650.0, 600.0, 400.0, 1280, 720);

        List<CalibrationObservation> observations = new java.util.ArrayList<>();
        for (int view = 0; view < poses.size(); view++) {
            Intrinsics cameraForView = view % 2 == 0 ? CalibrationFixtures.INTRINSICS : alternate;
            observations.add(CalibrationFixtures.observations(
                    projector, cameraForView, CalibrationFixtures.ZERO_DISTORTION,
                    board, List.of(poses.get(view))).get(0));
        }

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, false));

        // Refinement must have reduced the residual (it really optimized), but not to the
        // threshold — and the report has to say so explicitly rather than claiming success.
        assertTrue(result.convergence().finalRmsError()
                        < result.convergence().initialRmsError(),
                "best-fit refinement must still descend on inconsistent data");
        assertFalse(result.convergence().converged(),
                "observations no single camera can explain must not be reported as converged");
        assertEquals(StopReason.IMPROVEMENT_TOO_SMALL, result.convergence().stopReason());
        assertTrue(result.convergence().iterations() > 0);
        assertTrue(result.meanReprojectionError() > 1e-4,
                "the irreducible residual must be visible in the error report");
    }

    @Test
    void recoveredPosesReprojectTheBoardThroughTheReturnedModel() {        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        List<Extrinsics> truePoses = CalibrationFixtures.boardPoses();
        List<CalibrationObservation> observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, truePoses);

        CalibrationResult result = calibrator.calibrate(
                new CalibrationProblem(observations, 1280, 720, true));

        assertEquals(truePoses.size(), result.poses().size());
        for (int view = 0; view < truePoses.size(); view++) {
            assertPosesAgree(truePoses.get(view), result.poses().get(view));
        }
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    private static void assertGroundTruthIntrinsics(CalibrationResult result) {
        var k = CalibrationFixtures.INTRINSICS;
        assertEquals(k.fx(), result.intrinsics().fx(), PARAM_TOL, "fx mismatch");
        assertEquals(k.fy(), result.intrinsics().fy(), PARAM_TOL, "fy mismatch");
        assertEquals(k.cx(), result.intrinsics().cx(), PARAM_TOL, "cx mismatch");
        assertEquals(k.cy(), result.intrinsics().cy(), PARAM_TOL, "cy mismatch");
        assertEquals(k.width(), result.intrinsics().width());
        assertEquals(k.height(), result.intrinsics().height());
    }

    private static void assertReprojectionNearZero(CalibrationResult result) {
        assertTrue(result.maxReprojectionError() < REPROJECTION_TOL,
                "max reprojection error must be near zero, got " + result.maxReprojectionError());
        assertTrue(result.meanReprojectionError() < REPROJECTION_TOL,
                "mean reprojection error must be near zero, got " + result.meanReprojectionError());
    }

    private static boolean allDistortionNearZero(CalibrationResult result) {
        return Math.abs(result.distortion().k1()) < 1e-9
                && Math.abs(result.distortion().k2()) < 1e-9
                && Math.abs(result.distortion().p1()) < 1e-9
                && Math.abs(result.distortion().p2()) < 1e-9;
    }

    private static void assertPosesAgree(Extrinsics expected, Extrinsics actual) {
        for (int i = 0; i < 3; i++) {
            assertEquals(expected.translation()[i], actual.translation()[i], 1e-4,
                    "translation component " + i);
            for (int j = 0; j < 3; j++) {
                assertEquals(expected.rotation()[i][j], actual.rotation()[i][j], 1e-4,
                        "rotation entry [" + i + "][" + j + "]");
            }
        }
    }
}
