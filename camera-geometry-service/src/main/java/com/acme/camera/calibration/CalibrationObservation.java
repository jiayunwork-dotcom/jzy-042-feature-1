package com.acme.camera.calibration;

import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;

import java.util.List;

/**
 * One calibration observation: the same set of physically fixed control points, expressed in
 * the shared world frame, paired with the pixels at which they were observed in one image.
 * The two lists are index-aligned ({@code worldPoints[i]} was seen at {@code pixels[i]}).
 *
 * @param worldPoints control-point positions in the shared world coordinate frame
 * @param pixels      observed pixel positions, index-aligned with {@code worldPoints}
 */
public record CalibrationObservation(List<Point3D> worldPoints, List<Point2D> pixels) {

    public int pointCount() {
        return worldPoints.size();
    }
}
