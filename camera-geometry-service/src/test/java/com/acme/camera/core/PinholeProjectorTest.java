package com.acme.camera.core;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Geometric invariants of the projection pipeline, pinned down at the unit level.
 */
class PinholeProjectorTest {

    private final PinholeProjector projector = new PinholeProjector();

    private static final Intrinsics HD = new Intrinsics(800.0, 800.0, 640.0, 360.0, 1280, 720);
    private static final DistortionCoefficients DISTORTION =
            new DistortionCoefficients(-0.1, 0.01, 0.001, -0.002);

    private static final List<Point3D> SAMPLE_POINTS = List.of(
            new Point3D(0.3, -0.2, 2.0),
            new Point3D(-1.1, 0.4, 3.5),
            new Point3D(0.0, 0.0, 1.0),
            new Point3D(2.5, 1.5, 6.0),
            new Point3D(-0.05, -0.9, 1.2));

    @Test
    void zeroDistortionLeavesPixelsExactlyIdentical() {
        for (Point3D p : SAMPLE_POINTS) {
            ProjectionResult withZeroDistortion = projector.project(p, HD, DistortionCoefficients.ZERO);

            // Reference: pure pinhole with no distortion stage at all.
            double expectedU = HD.cx() + HD.fx() * (p.x() / p.z());
            double expectedV = HD.cy() + HD.fy() * (p.y() / p.z());
            assertEquals(expectedU, withZeroDistortion.u(), 1e-12, "u must be untouched by zero distortion");
            assertEquals(expectedV, withZeroDistortion.v(), 1e-12, "v must be untouched by zero distortion");

            // The distortion stage itself must be the identity for zero coefficients.
            Point2D normalized = new Point2D(p.x() / p.z(), p.y() / p.z());
            Point2D after = BrownConradyDistortion.distort(normalized, DistortionCoefficients.ZERO);
            assertEquals(normalized.x(), after.x(), 0.0);
            assertEquals(normalized.y(), after.y(), 0.0);
        }
    }

    @Test
    void pointsRecedingAlongOpticalAxisShrinkTowardPrincipalPoint() {
        Point3D base = new Point3D(0.7, -0.4, 2.0);
        double previousRadius = Double.MAX_VALUE;
        for (double dz : new double[]{0.0, 1.0, 3.0, 7.0, 15.0, 31.0}) {
            Point3D moved = new Point3D(base.x(), base.y(), base.z() + dz);
            ProjectionResult r = projector.project(moved, HD, DISTORTION);
            double radius = Math.hypot(r.u() - HD.cx(), r.v() - HD.cy());
            assertTrue(radius < previousRadius,
                    "image point must move closer to the principal point as Z grows (dz=" + dz + ")");
            previousRadius = radius;
        }
        // Very far away, the image point essentially coincides with the principal point.
        ProjectionResult far = projector.project(new Point3D(base.x(), base.y(), base.z() + 100_000.0), HD, DISTORTION);
        assertTrue(Math.hypot(far.u() - HD.cx(), far.v() - HD.cy()) < 0.5,
                "a point very far along Z must converge to the principal point");
    }

    @Test
    void doublingBothFocalLengthsDoublesRadiusFromPrincipalPoint() {
        Intrinsics doubled = new Intrinsics(HD.fx() * 2, HD.fy() * 2, HD.cx(), HD.cy(), HD.width(), HD.height());
        for (Point3D p : SAMPLE_POINTS) {
            // Holds both with and without distortion: distortion never touches fx/fy.
            for (DistortionCoefficients d : new DistortionCoefficients[]{DistortionCoefficients.ZERO, DISTORTION}) {
                ProjectionResult normal = projector.project(p, HD, d);
                ProjectionResult wide = projector.project(p, doubled, d);
                double r1 = Math.hypot(normal.u() - HD.cx(), normal.v() - HD.cy());
                double r2 = Math.hypot(wide.u() - HD.cx(), wide.v() - HD.cy());
                assertEquals(2.0 * r1, r2, 1e-9,
                        "doubling fx and fy must double the radius from the principal point");
            }
        }
    }

    @Test
    void normalizationDividesByZNotTheOtherWayRound() {
        // (X, Y, Z) = (1, 0, 2) → x_n = X/Z = 0.5 → u = cx + fx·0.5.
        ProjectionResult r = projector.project(new Point3D(1.0, 0.0, 2.0), HD, DistortionCoefficients.ZERO);
        assertEquals(640.0 + 800.0 * 0.5, r.u(), 1e-12);
        assertEquals(360.0, r.v(), 1e-12);
    }

    @Test
    void radialDistortionUsesNormalizedRadiusNotPixels() {
        // Normalized radius 0.1 → pixel radius 80 with fx=800. If r were (wrongly) taken
        // from pixels, k1·r² would be 0.5·6400 = 3200 instead of 0.5·0.01 = 0.005.
        DistortionCoefficients k1Only = new DistortionCoefficients(0.5, 0.0, 0.0, 0.0);
        Point2D distorted = BrownConradyDistortion.distort(new Point2D(0.1, 0.0), k1Only);
        assertEquals(0.1 * (1.0 + 0.5 * 0.01), distorted.x(), 1e-15,
                "radial distortion must be computed on the normalized plane");
    }

    @Test
    void boundsCheckReportsInsideAndOutsideCorrectly() {
        ProjectionResult inside = projector.project(new Point3D(0.1, 0.1, 2.0), HD, DistortionCoefficients.ZERO);
        assertTrue(inside.inBounds());

        // Far off-axis → outside the 1280x720 frame.
        ProjectionResult outside = projector.project(new Point3D(5.0, 0.0, 1.0), HD, DistortionCoefficients.ZERO);
        assertTrue(!outside.inBounds());
    }
}
