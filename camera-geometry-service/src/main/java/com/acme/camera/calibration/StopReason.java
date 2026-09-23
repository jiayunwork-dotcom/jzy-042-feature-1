package com.acme.camera.calibration;

/** Why the iterative refinement stopped — the machine-readable part of the convergence report. */
public enum StopReason {

    /** Residual RMS fell to/below the configured pixel threshold. */
    RESIDUAL_THRESHOLD_REACHED,
    /** The relative cost improvement between two accepted rounds became negligible. */
    IMPROVEMENT_TOO_SMALL,
    /** The parameter step itself became negligible in every component. */
    STEP_TOO_SMALL,
    /** The round budget was exhausted without reaching a threshold. */
    MAX_ITERATIONS_REACHED,
    /** The trust region shrank to its floor while the cost still could not be reduced. */
    DIVERGED
}
