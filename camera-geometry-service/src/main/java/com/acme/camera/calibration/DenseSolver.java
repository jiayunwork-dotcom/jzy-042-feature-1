package com.acme.camera.calibration;

/**
 * Small dense linear-algebra toolkit for the calibration optimizer (no external
 * dependencies): normal-equation assembly, Cholesky solve with adaptive diagonal jitter,
 * Jacobi eigendecomposition for symmetric matrices, and basic 3x3 eigen access used to
 * detect degenerate (rank-deficient / near-planar) configurations.
 */
public final class DenseSolver {

    private DenseSolver() {
    }

    /** Solve A·x = b for symmetric positive (semi-)definite A by Cholesky with diagonal jitter. */
    public static double[] choleskySolve(double[][] a, double[] b) {
        int n = a.length;
        double jitter = 0.0;
        double maxDiag = 1.0;
        for (int i = 0; i < n; i++) {
            maxDiag = Math.max(maxDiag, Math.abs(a[i][i]));
        }
        for (int attempt = 0; attempt < 60; attempt++) {
            double[][] m = new double[n][n];
            for (int i = 0; i < n; i++) {
                System.arraycopy(a[i], 0, m[i], 0, n);
                m[i][i] += jitter * maxDiag;
            }
            double[][] l = new double[n][n];
            boolean ok = true;
            for (int i = 0; i < n && ok; i++) {
                for (int j = 0; j <= i; j++) {
                    double sum = m[i][j];
                    for (int k = 0; k < j; k++) {
                        sum -= l[i][k] * l[j][k];
                    }
                    if (i == j) {
                        if (sum <= 1e-18 * maxDiag) {
                            ok = false;
                            break;
                        }
                        l[i][j] = Math.sqrt(sum);
                    } else {
                        l[i][j] = sum / l[j][j];
                    }
                }
            }
            if (ok) {
                return solveLowerTriangular(l, b);
            }
            jitter = jitter == 0.0 ? 1e-14 : jitter * 10.0;
        }
        return null;
    }

    private static double[] solveLowerTriangular(double[][] l, double[] b) {
        int n = l.length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = b[i];
            for (int k = 0; k < i; k++) {
                sum -= l[i][k] * y[k];
            }
            y[i] = sum / l[i][i];
        }
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = y[i];
            for (int k = i + 1; k < n; k++) {
                sum -= l[k][i] * x[k];
            }
            x[i] = sum / l[i][i];
        }
        return x;
    }

    /**
     * Jacobi eigendecomposition of a real symmetric matrix. Eigenvalues come back
     * ascending; columns of the returned matrix are the matching eigenvectors.
     */
    public static double[][] sortedJacobi(double[][] matrix, double[] eigenvalues) {
        int n = matrix.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) {
            a[i] = matrix[i].clone();
        }
        double[][] v = identity(n);
        for (int sweep = 0; sweep < 100; sweep++) {
            double off = 0.0;
            for (int p = 0; p < n - 1; p++) {
                for (int q = p + 1; q < n; q++) {
                    off += a[p][q] * a[p][q];
                }
            }
            if (off <= 1e-26) {
                break;
            }
            for (int p = 0; p < n - 1; p++) {
                for (int q = p + 1; q < n; q++) {
                    double apq = a[p][q];
                    if (apq == 0.0) {
                        continue;
                    }
                    double app = a[p][p];
                    double aqq = a[q][q];
                    double phi = 0.5 * Math.atan2(2.0 * apq, aqq - app);
                    double c = Math.cos(phi);
                    double s = Math.sin(phi);
                    for (int k = 0; k < n; k++) {
                        if (k == p || k == q) {
                            continue;
                        }
                        double akp = a[k][p];
                        double akq = a[k][q];
                        a[k][p] = c * akp - s * akq;
                        a[p][k] = a[k][p];
                        a[k][q] = s * akp + c * akq;
                        a[q][k] = a[k][q];
                    }
                    a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
                    a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
                    a[p][q] = 0.0;
                    a[q][p] = 0.0;
                    for (int k = 0; k < n; k++) {
                        double vkp = v[k][p];
                        double vkq = v[k][q];
                        v[k][p] = c * vkp - s * vkq;
                        v[k][q] = s * vkp + c * vkq;
                    }
                }
            }
        }
        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) {
            order[i] = i;
            eigenvalues[i] = a[i][i];
        }
        java.util.Arrays.sort(order, (i, j) -> Double.compare(eigenvalues[i], eigenvalues[j]));
        double[][] sortedVectors = new double[n][n];
        double[] sortedValues = new double[n];
        for (int col = 0; col < n; col++) {
            int src = order[col];
            sortedValues[col] = eigenvalues[src];
            for (int row = 0; row < n; row++) {
                sortedVectors[row][col] = v[row][src];
            }
        }
        System.arraycopy(sortedValues, 0, eigenvalues, 0, n);
        return sortedVectors;
    }

    private static double[][] identity(int n) {
        double[][] v = new double[n][n];
        for (int i = 0; i < n; i++) {
            v[i][i] = 1.0;
        }
        return v;
    }
}
