package com.acme.camera.core;

import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;

import java.util.Map;

/**
 * Pinhole projection with Brown-Conrady distortion. This is THE projection function of the
 * service: the single-point endpoint and the batch projection job both call
 * {@link #project(Point3D, Intrinsics, DistortionCoefficients)}, so the same 3D point always
 * yields the same pixel regardless of which entry point was used.
 *
 * <p>Pipeline (order matters):
 * <ol>
 *   <li>Normalize the camera-frame point onto the z=1 plane: x = X/Z, y = Y/Z.</li>
 *   <li>Apply radial + tangential distortion ON THE NORMALIZED PLANE
 *       (never on pixels — see {@link BrownConradyDistortion}).</li>
 *   <li>Map to pixels through the intrinsics: u = cx + fx·x_d, v = cy + fy·y_d.</li>
 *   <li>Report whether the pixel lies inside the image: 0 &lt;= u &lt; width, 0 &lt;= v &lt; height.</li>
 * </ol>
 */
public class PinholeProjector {

    /**
     * Project a camera-frame 3D point to a pixel.
     *
     * @throws GeometryValidationException with type {@code POINT_BEHIND_CAMERA} when Z &lt;= 0
     *                                     (a point at or behind the camera cannot be projected)
     */
    public ProjectionResult project(Point3D cameraPoint, Intrinsics intrinsics, DistortionCoefficients distortion) {
        double z = cameraPoint.z();
        if (!Double.isFinite(z) || z <= 0.0) {
            throw new GeometryValidationException(
                    ErrorType.POINT_BEHIND_CAMERA,
                    "Point has Z=" + z + " in camera coordinates; only points in front of the camera (Z > 0) can be projected",
                    Map.of("z", z));
        }
        // Step 1: normalized plane.
        double xn = cameraPoint.x() / z;
        double yn = cameraPoint.y() / z;

        // Step 2: distortion on the normalized plane.
        Point2D distorted = BrownConradyDistortion.distort(new Point2D(xn, yn), distortion);

        // Step 3: intrinsics to pixels.
        double u = intrinsics.cx() + intrinsics.fx() * distorted.x();
        double v = intrinsics.cy() + intrinsics.fy() * distorted.y();

        // Step 4: image-bounds check.
        boolean inBounds = u >= 0.0 && u < intrinsics.width() && v >= 0.0 && v < intrinsics.height();
        return new ProjectionResult(u, v, inBounds);
    }
}
