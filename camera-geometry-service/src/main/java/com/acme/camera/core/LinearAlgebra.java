package com.acme.camera.core;

/**
 * Minimal dense linear algebra (no external dependencies): the cyclic Jacobi
 * eigendecomposition for small real symmetric matrices. Used by the DLT triangulator
 * to extract the null vector of a 4x4 normal-equations matrix.
 */
public final class LinearAlgebra {

    private LinearAlgebra() {
    }

    /**
     * Eigendecomposition of a real symmetric n×n matrix via cyclic Jacobi rotations.
     *
     * @param a           symmetric matrix (not modified)
     * @param eigenvalues output array of length n, filled with the eigenvalues
     * @return eigenvectors stored by column: {@code result[row][col]} is component
     *         {@code row} of the eigenvector belonging to {@code eigenvalues[col]}
     */
    public static double[][] jacobiEigen(double[][] a, double[] eigenvalues) {
        int n = a.length;
        double[][] m = new double[n][];
        for (int i = 0; i < n; i++) {
            m[i] = a[i].clone();
        }
        double[][] v = new double[n][n];
        for (int i = 0; i < n; i++) {
            v[i][i] = 1.0;
        }

        for (int sweep = 0; sweep < 100; sweep++) {
            double offDiagonalNorm = 0.0;
            for (int p = 0; p < n - 1; p++) {
                for (int q = p + 1; q < n; q++) {
                    offDiagonalNorm += m[p][q] * m[p][q];
                }
            }
            if (offDiagonalNorm <= 1e-24) {
                break;
            }
            for (int p = 0; p < n - 1; p++) {
                for (int q = p + 1; q < n; q++) {
                    double apq = m[p][q];
                    if (apq == 0.0) {
                        continue;
                    }
                    double app = m[p][p];
                    double aqq = m[q][q];
                    // Rotation angle that annihilates m[p][q]: tan(2φ) = 2·apq / (aqq − app).
                    double phi = 0.5 * Math.atan2(2.0 * apq, aqq - app);
                    double c = Math.cos(phi);
                    double s = Math.sin(phi);

                    for (int k = 0; k < n; k++) {
                        if (k == p || k == q) {
                            continue;
                        }
                        double mkp = m[k][p];
                        double mkq = m[k][q];
                        m[k][p] = c * mkp - s * mkq;
                        m[p][k] = m[k][p];
                        m[k][q] = s * mkp + c * mkq;
                        m[q][k] = m[k][q];
                    }
                    m[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
                    m[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
                    m[p][q] = 0.0;
                    m[q][p] = 0.0;

                    for (int k = 0; k < n; k++) {
                        double vkp = v[k][p];
                        double vkq = v[k][q];
                        v[k][p] = c * vkp - s * vkq;
                        v[k][q] = s * vkp + c * vkq;
                    }
                }
            }
        }
        for (int i = 0; i < n; i++) {
            eigenvalues[i] = m[i][i];
        }
        return v;
    }
}
