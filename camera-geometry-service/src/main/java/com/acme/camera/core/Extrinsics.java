package com.acme.camera.core;

/**
 * Rigid world-to-camera transform: X_cam = R * X_world + t.
 *
 * @param rotation    3x3 rotation matrix (rows)
 * @param translation 3-vector
 */
public record Extrinsics(double[][] rotation, double[] translation) {

    /** Transform a world-frame point into the camera frame. */
    public Point3D transform(Point3D world) {
        double[][] r = rotation;
        return new Point3D(
                r[0][0] * world.x() + r[0][1] * world.y() + r[0][2] * world.z() + translation[0],
                r[1][0] * world.x() + r[1][1] * world.y() + r[1][2] * world.z() + translation[1],
                r[2][0] * world.x() + r[2][1] * world.y() + r[2][2] * world.z() + translation[2]);
    }
}
