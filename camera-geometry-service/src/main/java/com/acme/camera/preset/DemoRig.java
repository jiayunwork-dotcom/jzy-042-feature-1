package com.acme.camera.preset;

import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.Point3D;

import java.util.ArrayList;
import java.util.List;

/**
 * The known calibration fixture shipped with the service: a unit cube centered at the
 * world origin, observed by two cameras. All eight corners project inside the image in
 * both views, so the fixture can be eyeballed for correctness.
 *
 * <p>Camera 1 sits at (0, 0, 5) looking at the origin (image x = world +x, image y = world −y).
 * Camera 2 sits at (2, 1, 4) looking at the origin. Both rotations are proper (det = +1):
 * <pre>
 *   R1 = [1 0 0; 0 -1 0; 0 0 -1]                     t1 = (0, 0, 5)
 *   R2 = [2/√5  0  -1/√5;  1/√105  -10/√105  2/√105;  -2/√21  -1/√21  -4/√21]
 *                                                  t2 = (0, 0, √21)
 * </pre>
 */
public final class DemoRig {

    private DemoRig() {
    }

    public static final Intrinsics INTRINSICS = new Intrinsics(800.0, 800.0, 640.0, 360.0, 1280, 720);

    public static final DistortionCoefficients DISTORTION =
            new DistortionCoefficients(-0.02, 0.0005, 0.0003, -0.0002);

    public static final Extrinsics CAMERA1 = new Extrinsics(
            new double[][]{
                    {1.0, 0.0, 0.0},
                    {0.0, -1.0, 0.0},
                    {0.0, 0.0, -1.0}},
            new double[]{0.0, 0.0, 5.0});

    public static final Extrinsics CAMERA2 = new Extrinsics(
            new double[][]{
                    {0.8944271910, 0.0, -0.4472135955},
                    {0.0975900073, -0.9759000729, 0.1951800146},
                    {-0.4364357805, -0.2182178902, -0.8728715609}},
            new double[]{0.0, 0.0, 4.5825756950});

    /** The eight corners of the axis-aligned unit cube centered at the world origin. */
    public static List<Point3D> cubeCorners() {
        List<Point3D> corners = new ArrayList<>(8);
        for (double x : new double[]{-0.5, 0.5}) {
            for (double y : new double[]{-0.5, 0.5}) {
                for (double z : new double[]{-0.5, 0.5}) {
                    corners.add(new Point3D(x, y, z));
                }
            }
        }
        return corners;
    }
}
