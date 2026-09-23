package com.acme.camera.calibration;

/**
 * RQ decomposition of a 3x3 matrix A = R Q with R upper triangular and Q orthogonal,
 * used to split a DLT camera matrix P into intrinsics (upper triangular K) and rotation.
 *
 * <p>Implemented by column-wise modified Gram-Schmidt on {@code Aᵀ}: Aᵀ = Q Rᵀ where Rᵀ is
 * lower triangular, hence A = R Q exactly, with no assumption about the sign of the
 * diagonal entries.
 */
public final class RqDecomposition {

    private RqDecomposition() {
    }

    /** Result of {@link #decompose3x3(double[][])}: {@code a = r.times(q)}. */
    public record Factors(double[][] r, double[][] q) {
    }

    public static Factors decompose3x3(double[][] a) {
        // Columns of Aᵀ are the rows of A.
        double[][] cols = new double[][]{
                {a[0][0], a[1][0], a[2][0]},
                {a[0][1], a[1][1], a[2][1]},
                {a[0][2], a[1][2], a[2][2]}};

        double[][] qCols = new double[3][];
        double[][] lower = new double[3][3];
        for (int k = 0; k < 3; k++) {
            double[] v = cols[k].clone();
            for (int j = 0; j < k; j++) {
                double dot = dot(cols[k], qCols[j]);
                lower[k][j] = dot;
                v = minus(v, scale(qCols[j], dot));
            }
            double norm = Math.sqrt(dot(v, v));
            lower[k][k] = norm;
            qCols[k] = scale(v, 1.0 / norm);
        }

        // Aᵀ = Qcols · L  →  A = Lᵀ · Qcolsᵀ.
        double[][] r = new double[3][3];
        double[][] q = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = lower[j][i];
                q[i][j] = qCols[j][i];
            }
        }
        return new Factors(r, q);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] scale(double[] a, double s) {
        return new double[]{a[0] * s, a[1] * s, a[2] * s};
    }

    private static double[] minus(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
}
