package com.acme.camera.calibration;

import java.util.ArrayList;
import java.util.List;

/**
 * Generic Levenberg–Marquardt refinement for dense nonlinear least squares, isolated from
 * all camera-calibration specifics (the model is injected via {@link LmResidualModel}).
 *
 * <p>The two-stage nature of calibration lives elsewhere: callers supply a linear/closed-form
 * initial parameter vector; this class performs ONLY the iterative minimization of the sum
 * of squared residuals. Jacobians are estimated by central finite differences, which keeps
 * the derivative logic independent of the parameter packing and trivially consistent with
 * the residual actually evaluated.
 *
 * <p>Stopping conditions (explicit and machine-readable):
 * <ol>
 *   <li>RMS residual at/below {@code rmsThreshold} → {@link StopReason#RESIDUAL_THRESHOLD_REACHED},</li>
 *   <li>relative cost improvement between two accepted rounds below {@code improvementTolerance}
 *       → {@link StopReason#IMPROVEMENT_TOO_SMALL},</li>
 *   <li>the accepted parameter step below {@code stepTolerance} (relative, per component)
 *       → {@link StopReason#STEP_TOO_SMALL},</li>
 *   <li>{@code maxIterations} rounds exhausted → {@link StopReason#MAX_ITERATIONS_REACHED},</li>
 *   <li>the trust-region damping grows past its floor without ever reducing the cost
 *       → {@link StopReason#DIVERGED}.</li>
 * </ol>
 */
public final class LevenbergMarquardt {

    private static final double FINITE_DIFFERENCE_EPS = 1e-7;
    private static final double INITIAL_DAMPING = 1e-3;
    private static final double DAMPING_UP = 10.0;
    private static final double DAMPING_DOWN = 0.3;
    private static final double MAX_DAMPING = 1e14;

    private final int maxIterations;
    private final double rmsThreshold;
    private final double improvementTolerance;
    private final double stepTolerance;

    public LevenbergMarquardt(int maxIterations,
                              double rmsThreshold,
                              double improvementTolerance,
                              double stepTolerance) {
        this.maxIterations = maxIterations;
        this.rmsThreshold = rmsThreshold;
        this.improvementTolerance = improvementTolerance;
        this.stepTolerance = stepTolerance;
    }

    public LmOutcome optimize(LmResidualModel model, double[] initial) {
        return optimize(model, initial, null);
    }

    /**
     * Minimize {@code model} starting from {@code initial}.
     *
     * @param active when non-null, only indices with {@code active[i]} are free parameters;
     *               all other entries stay fixed (used for pose-only initialization passes)
     */
    public LmOutcome optimize(LmResidualModel model, double[] initial, boolean[] active) {
        int n = model.parameterCount();
        int m = model.residualCount();
        double[] params = initial.clone();

        double[] residuals = model.residuals(params);
        if (!allFinite(residuals)) {
            // The initialization itself is geometrically impossible; nothing to refine.
            return new LmOutcome(params, StopReason.DIVERGED, 0,
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        double cost = sumSquares(residuals);
        double damping = INITIAL_DAMPING;

        List<Integer> freeIndices = new ArrayList<>();
        for (int j = 0; j < n; j++) {
            if (active == null || active[j]) {
                freeIndices.add(j);
            }
        }
        int f = freeIndices.size();

        for (int iteration = 1; iteration <= maxIterations; iteration++) {
            double rms = Math.sqrt(cost / m);
            if (rms <= rmsThreshold) {
                return new LmOutcome(params, StopReason.RESIDUAL_THRESHOLD_REACHED,
                        iteration - 1, cost, rms);
            }

            // Jacobian columns for the free coordinates via central differences.
            double[][] jacobian = new double[f][m];
            boolean differentiable = true;
            for (int col = 0; col < f; col++) {
                int j = freeIndices.get(col);
                double h = finiteDifferenceStep(params[j]);
                double saved = params[j];
                params[j] = saved + h;
                double[] forward = model.residuals(params);
                params[j] = saved - h;
                double[] backward = model.residuals(params);
                params[j] = saved;
                if (!allFinite(forward) || !allFinite(backward)) {
                    differentiable = false;
                    break;
                }
                double twoH = 2.0 * h;
                for (int i = 0; i < m; i++) {
                    jacobian[col][i] = (forward[i] - backward[i]) / twoH;
                }
            }
            if (!differentiable) {
                // A trial crossed into an invalid region even at the current point; enlarge the
                // trust region bias toward steepest descent and retry within the round budget.
                damping *= DAMPING_UP;
                if (damping > MAX_DAMPING) {
                    return new LmOutcome(params, StopReason.DIVERGED,
                            iteration - 1, cost, Math.sqrt(cost / m));
                }
                continue;
            }

            double[][] jtj = new double[f][f];
            double[] jtf = new double[f];
            for (int a = 0; a < f; a++) {
                for (int i = 0; i < m; i++) {
                    jtf[a] -= jacobian[a][i] * residuals[i];
                }
                for (int b = a; b < f; b++) {
                    double s = 0.0;
                    for (int i = 0; i < m; i++) {
                        s += jacobian[a][i] * jacobian[b][i];
                    }
                    jtj[a][b] = s;
                    jtj[b][a] = s;
                }
            }

            double acceptedDamping = damping;
            while (acceptedDamping <= MAX_DAMPING) {
                double[][] damped = new double[f][f];
                for (int a = 0; a < f; a++) {
                    System.arraycopy(jtj[a], 0, damped[a], 0, f);
                    damped[a][a] += acceptedDamping * jtj[a][a] + acceptedDamping * 1e-12;
                }
                double[] freeStep = DenseSolver.choleskySolve(damped, jtf);
                if (freeStep == null) {
                    acceptedDamping *= DAMPING_UP;
                    continue;
                }
                double[] trial = params.clone();
                for (int a = 0; a < f; a++) {
                    trial[freeIndices.get(a)] += freeStep[a];
                }
                double[] trialResiduals = model.residuals(trial);
                if (!allFinite(trialResiduals)) {
                    acceptedDamping *= DAMPING_UP;
                    continue;
                }
                double trialCost = sumSquares(trialResiduals);
                if (trialCost < cost) {
                    // Accept the step.
                    double relativeImprovement = (cost - trialCost) / Math.max(cost, 1e-30);
                    boolean stepSmall = true;
                    for (int a = 0; a < f; a++) {
                        int j = freeIndices.get(a);
                        double scale = Math.max(1.0, Math.abs(params[j]));
                        if (Math.abs(freeStep[a]) / scale > stepTolerance) {
                            stepSmall = false;
                            break;
                        }
                    }
                    params = trial;
                    residuals = trialResiduals;
                    cost = trialCost;
                    damping = Math.max(1e-12, acceptedDamping * DAMPING_DOWN);

                    if (relativeImprovement < improvementTolerance) {
                        return new LmOutcome(params, StopReason.IMPROVEMENT_TOO_SMALL,
                                iteration, cost, Math.sqrt(cost / m));
                    }
                    if (stepSmall) {
                        return new LmOutcome(params, StopReason.STEP_TOO_SMALL,
                                iteration, cost, Math.sqrt(cost / m));
                    }
                    break;
                }
                acceptedDamping *= DAMPING_UP;
            }

            if (acceptedDamping > MAX_DAMPING) {
                return new LmOutcome(params, StopReason.DIVERGED,
                        iteration, cost, Math.sqrt(cost / m));
            }
        }

        return new LmOutcome(params, StopReason.MAX_ITERATIONS_REACHED,
                maxIterations, cost, Math.sqrt(cost / m));
    }

    private static double finiteDifferenceStep(double value) {
        return Math.max(FINITE_DIFFERENCE_EPS, FINITE_DIFFERENCE_EPS * Math.abs(value));
    }

    private static double sumSquares(double[] r) {
        double s = 0.0;
        for (double v : r) {
            s += v * v;
        }
        return s;
    }

    private static boolean allFinite(double[] r) {
        for (double v : r) {
            if (!Double.isFinite(v)) {
                return false;
            }
        }
        return true;
    }
}
