package com.acme.camera.core;

/**
 * Triangulation of matched two-view pixel correspondences by the
 * <b>Direct Linear Transform (DLT)</b> — the single, fixed triangulation method of this
 * service (chosen over the midpoint method because it minimizes the algebraic reprojection
 * residual and generalizes cleanly to noisy real-world matches).
 *
 * <p>Per matched pair:
 * <ol>
 *   <li>Each pixel is mapped back to the normalized plane and undistorted
 *       ({@link BrownConradyDistortion#undistort}), so both views become ideal pinhole rays.</li>
 *   <li>Each view contributes two equations from its pose matrix [R|t] (intrinsics are
 *       already divided out by step 1): {@code x·P3 − P1 = 0} and {@code y·P3 − P2 = 0}.</li>
 *   <li>The homogeneous 3D point is the least-squares null vector of the stacked 4×4
 *       system, extracted as the eigenvector of the smallest eigenvalue of AᵀA
 *       (Jacobi eigendecomposition, see {@link LinearAlgebra}).</li>
 * </ol>
 *
 * <p>The two views' pixel coordinates are NEVER averaged to fake a 3D point; the 3D point
 * comes solely from intersecting the two back-projected rays in the least-squares sense.
 */
public class DltTriangulator {

    /**
     * Triangulate one matched pair into a world-frame 3D point.
     *
     * @param pixel1 matched pixel in view 1
     * @param cam1   camera 1 (intrinsics + distortion + extrinsics)
     * @param pixel2 matched pixel in view 2
     * @param cam2   camera 2
     * @return world-frame 3D point
     */
    public Point3D triangulate(Point2D pixel1, CameraModel cam1, Point2D pixel2, CameraModel cam2) {
        double[][] ata = new double[4][4];
        accumulateView(ata, poseMatrix(cam1), undistortedNormalized(pixel1, cam1));
        accumulateView(ata, poseMatrix(cam2), undistortedNormalized(pixel2, cam2));

        double[] eigenvalues = new double[4];
        double[][] eigenvectors = LinearAlgebra.jacobiEigen(ata, eigenvalues);
        int smallest = 0;
        for (int i = 1; i < 4; i++) {
            if (eigenvalues[i] < eigenvalues[smallest]) {
                smallest = i;
            }
        }
        double w = eigenvectors[3][smallest];
        return new Point3D(
                eigenvectors[0][smallest] / w,
                eigenvectors[1][smallest] / w,
                eigenvectors[2][smallest] / w);
    }

    /** Pixel → normalized plane → undistorted normalized coordinates (ideal pinhole ray). */
    private static Point2D undistortedNormalized(Point2D pixel, CameraModel cam) {
        Intrinsics k = cam.intrinsics();
        Point2D normalized = new Point2D(
                (pixel.x() - k.cx()) / k.fx(),
                (pixel.y() - k.cy()) / k.fy());
        return BrownConradyDistortion.undistort(normalized, cam.distortion());
    }

    /**
     * Pose matrix [R | t] (3×4). Because matched pixels are converted to undistorted
     * normalized coordinates before triangulation, the intrinsics must NOT be applied
     * here — the normalized plane already has K divided out.
     */
    static double[][] poseMatrix(CameraModel cam) {
        double[][] r = cam.extrinsics().rotation();
        double[] t = cam.extrinsics().translation();
        return new double[][]{
                {r[0][0], r[0][1], r[0][2], t[0]},
                {r[1][0], r[1][1], r[1][2], t[1]},
                {r[2][0], r[2][1], r[2][2], t[2]}};
    }

    /**
     * Add one view's two DLT equations to the normal-equations matrix AᵀA:
     * x·P3 − P1 = 0 and y·P3 − P2 = 0, where (x, y) is the undistorted normalized pixel.
     */
    private static void accumulateView(double[][] ata, double[][] p, Point2D normalizedPixel) {
        double x = normalizedPixel.x();
        double y = normalizedPixel.y();
        double[] row1 = new double[4];
        double[] row2 = new double[4];
        for (int j = 0; j < 4; j++) {
            row1[j] = x * p[2][j] - p[0][j];
            row2[j] = y * p[2][j] - p[1][j];
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                ata[i][j] += row1[i] * row1[j] + row2[i] * row2[j];
            }
        }
    }
}
