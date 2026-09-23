package com.acme.camera.calibration;

import com.acme.camera.api.dto.CalibrationObservationDto;
import com.acme.camera.api.dto.Point2DDto;
import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;
import com.acme.camera.core.ProjectionResult;

import java.util.ArrayList;
import java.util.List;

/**
 * Shared calibration test fixtures: control-point geometry and ground-truth cameras, with
 * observations produced strictly through the service's own {@link PinholeProjector} so a
 * test's "ground truth" and the kernel under test share one projection definition.
 */
public final class CalibrationFixtures {

    public static final Intrinsics INTRINSICS = new Intrinsics(830.0, 820.0, 646.0, 355.0, 1280, 720);
    public static final DistortionCoefficients ZERO_DISTORTION = DistortionCoefficients.ZERO;
    public static final DistortionCoefficients DISTORTION =
            new DistortionCoefficients(-0.08, 0.02, 0.001, -0.002);

    private CalibrationFixtures() {
    }

    /** A flat {@code cols × rows} board in the world z=0 plane, centred at the origin. */
    public static List<Point3D> boardPoints(int rows, int cols, double spacing) {
        List<Point3D> points = new ArrayList<>(rows * cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                points.add(new Point3D(
                        (c - (cols - 1) / 2.0) * spacing,
                        (r - (rows - 1) / 2.0) * spacing,
                        0.0));
            }
        }
        return points;
    }

    /** A genuinely three-dimensional set of control points spanning a volume. */
    public static List<Point3D> volumePoints() {
        List<Point3D> points = new ArrayList<>();
        double[][] coords = {
                {-0.6, -0.5, -0.3}, {0.6, -0.5, -0.3}, {0.6, 0.5, -0.3}, {-0.6, 0.5, -0.3},
                {-0.5, -0.4, 0.0}, {0.5, -0.4, 0.0}, {0.5, 0.4, 0.0}, {-0.5, 0.4, 0.0},
                {-0.4, -0.3, 0.4}, {0.4, -0.3, 0.4}, {0.4, 0.3, 0.4}, {-0.4, 0.3, 0.4},
        };
        for (double[] c : coords) {
            points.add(new Point3D(c[0], c[1], c[2]));
        }
        return points;
    }

    public static Extrinsics pose(double rx, double ry, double rz, double tx, double ty, double tz) {
        return new Extrinsics(RotationMath.rodriguesToMatrix(rx, ry, rz),
                new double[]{tx, ty, tz});
    }

    /** A varied spread of board poses (different axes and distances). */
    public static List<Extrinsics> boardPoses() {
        return List.of(
                pose(0.15, -0.20, 0.05, 0.20, -0.10, 3.0),
                pose(-0.10, 0.25, 0.00, -0.30, 0.15, 3.2),
                pose(0.05, 0.10, -0.15, 0.10, 0.25, 2.9),
                pose(0.20, 0.05, 0.10, -0.15, -0.20, 3.1),
                pose(-0.18, -0.12, 0.08, 0.25, 0.05, 3.3));
    }

    /** Poses for the 3D volume rig. */
    public static List<Extrinsics> volumePoses() {
        return List.of(
                pose(0.10, -0.15, 0.00, 0.00, 0.00, 4.0),
                pose(-0.12, 0.10, 0.05, -0.50, 0.20, 4.3),
                pose(0.08, 0.20, -0.10, 0.40, -0.10, 4.1),
                pose(0.20, -0.05, 0.12, 0.20, 0.30, 4.5));
    }

    /** Project every world point through each pose, producing one observation per pose. */
    public static List<CalibrationObservation> observations(PinholeProjector projector,
                                                     Intrinsics intrinsics,
                                                     DistortionCoefficients distortion,
                                                     List<Point3D> worldPoints,
                                                     List<Extrinsics> poses) {
        List<CalibrationObservation> observations = new ArrayList<>(poses.size());
        for (Extrinsics pose : poses) {
            List<Point2D> pixels = new ArrayList<>(worldPoints.size());
            for (Point3D world : worldPoints) {
                ProjectionResult projected = projector.project(
                        pose.transform(world), intrinsics, distortion);
                pixels.add(new Point2D(projected.u(), projected.v()));
            }
            observations.add(new CalibrationObservation(worldPoints, pixels));
        }
        return observations;
    }

    /** Wire-DTO form of {@link #observations} for API-level tests. */
    public static List<CalibrationObservationDto> observationDtos(List<CalibrationObservation> observations) {
        List<CalibrationObservationDto> dtos = new ArrayList<>(observations.size());
        for (CalibrationObservation observation : observations) {
            List<Point3DDto> world = new ArrayList<>(observation.pointCount());
            List<Point2DDto> pixels = new ArrayList<>(observation.pointCount());
            for (Point3D p : observation.worldPoints()) {
                world.add(new Point3DDto(p.x(), p.y(), p.z()));
            }
            for (Point2D p : observation.pixels()) {
                pixels.add(new Point2DDto(p.x(), p.y()));
            }
            dtos.add(new CalibrationObservationDto(world, pixels));
        }
        return dtos;
    }
}
