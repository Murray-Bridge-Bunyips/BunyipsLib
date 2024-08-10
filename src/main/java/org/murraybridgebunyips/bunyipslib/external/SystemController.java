package org.murraybridgebunyips.bunyipslib.external;

/**
 * Marker interface that represents a control algorithm (PID, feedforward, etc).
 */
public interface SystemController {
    /**
     * Sets the coefficients for this controller.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    void setCoefficients(double... coeffs);

    /**
     * Calculate the next output of this control algorithm.
     *
     * @param process  the current value or first setpoint
     * @param setpoint the expected value or second setpoint
     * @return process variable that should be applied to the system
     */
    double calculate(double process, double setpoint);

    /**
     * Reset this controller back to an un-accumulated state, if applicable.
     */
    void reset();
}
