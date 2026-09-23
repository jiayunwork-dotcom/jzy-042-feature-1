package com.acme.camera.calibration;

/**
 * A nonlinear least-squares model for {@link LevenbergMarquardt}: given a parameter vector,
 * return the residual vector being driven to zero. Keeping this an interface (rather than
 * calibration-specific code inside the optimizer) isolates parameter pack/unpack concerns
 * from the iteration mechanics.
 */
public interface LmResidualModel {

    int parameterCount();

    int residualCount();

    /** Residuals for a parameter vector; non-finite entries signal a physically invalid trial. */
    double[] residuals(double[] parameters);
}
