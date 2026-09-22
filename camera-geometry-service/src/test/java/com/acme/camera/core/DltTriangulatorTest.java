package com.acme.camera.core;

import com.acme.camera.preset.DemoRig;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Round-trip invariant on the known cube fixture: project the corners with both demo
 * cameras, triangulate the matched pairs, and the recovered 3D points must reproject
 * back onto the original pixels with sub-micro-pixel error.
 */
class DltTriangulatorTest {

    private final PinholeProjector projector = new PinholeProjector();
    private final DltTriangulator triangulator = new DltTriangulator();

    private static final CameraModel CAM1 =
            new CameraModel(DemoRig.INTRINSICS, DemoRig.DISTORTION, DemoRig.CAMERA1);
    private static final CameraModel CAM2 =
            new CameraModel(DemoRig.INTRINSICS, DemoRig.DISTORTION, DemoRig.CAMERA2);

    @Test
    void cubeCornersTriangulateAndReprojectBackOntoOriginalPixels() {
        List<Point3D> corners = DemoRig.cubeCorners();
        for (int i = 0; i < corners.size(); i++) {
            Point3D corner = corners.get(i);

            // Simulate the matched pair as a real stereo pipeline would observe it.
            ProjectionResult px1 = projector.project(CAM1.extrinsics().transform(corner),
                    CAM1.intrinsics(), CAM1.distortion());
            ProjectionResult px2 = projector.project(CAM2.extrinsics().transform(corner),
                    CAM2.intrinsics(), CAM2.distortion());

            Point3D recovered = triangulator.triangulate(
                    new Point2D(px1.u(), px1.v()), CAM1, new Point2D(px2.u(), px2.v()), CAM2);

            // The 3D point itself must come back.
            assertEquals(corner.x(), recovered.x(), 1e-7, "corner " + i + " x");
            assertEquals(corner.y(), recovered.y(), 1e-7, "corner " + i + " y");
            assertEquals(corner.z(), recovered.z(), 1e-7, "corner " + i + " z");

            // And it must reproject onto the original pixels in BOTH views.
            ProjectionResult re1 = projector.project(CAM1.extrinsics().transform(recovered),
                    CAM1.intrinsics(), CAM1.distortion());
            ProjectionResult re2 = projector.project(CAM2.extrinsics().transform(recovered),
                    CAM2.intrinsics(), CAM2.distortion());
            double err1 = Math.hypot(re1.u() - px1.u(), re1.v() - px1.v());
            double err2 = Math.hypot(re2.u() - px2.u(), re2.v() - px2.v());
            assertTrue(err1 < 1e-6, "view 1 reprojection error " + err1 + " px for corner " + i);
            assertTrue(err2 < 1e-6, "view 2 reprojection error " + err2 + " px for corner " + i);
        }
    }

    @Test
    void demoCamerasSeeAllCubeCornersInsideTheImage() {
        for (Point3D corner : DemoRig.cubeCorners()) {
            ProjectionResult inCam1 = projector.project(CAM1.extrinsics().transform(corner),
                    CAM1.intrinsics(), CAM1.distortion());
            ProjectionResult inCam2 = projector.project(CAM2.extrinsics().transform(corner),
                    CAM2.intrinsics(), CAM2.distortion());
            assertTrue(inCam1.inBounds(), "corner must be inside the image in camera 1");
            assertTrue(inCam2.inBounds(), "corner must be inside the image in camera 2");
        }
    }

    @Test
    void undistortInvertsDistort() {
        Point2D normalized = new Point2D(0.12, -0.07);
        Point2D distorted = BrownConradyDistortion.distort(normalized, DemoRig.DISTORTION);
        Point2D recovered = BrownConradyDistortion.undistort(distorted, DemoRig.DISTORTION);
        assertEquals(normalized.x(), recovered.x(), 1e-12);
        assertEquals(normalized.y(), recovered.y(), 1e-12);
    }
}
