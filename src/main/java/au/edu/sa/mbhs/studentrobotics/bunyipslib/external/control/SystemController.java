package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control;

import androidx.annotation.NonNull;

import java.util.Arrays;

/**
 * Marker interface that represents a control algorithm (PID, feedforward, etc).
 * <p>
 * This interface may not always represent the same behaviour in different contexts, such as open-loop and closed
 * loop control. Therefore, it is important to validate which controllers are being used within a control system
 * as the protection of type-safety may cause an invalid controller to be used against one that expects a certain
 * type of response the controller is not able to provide.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public interface SystemController {
    /**
     * Gets the current coefficients for this controller.
     * The return of this method is expected to be in the same order and length as the set method.
     *
     * @return an array of coefficients for this controller
     */
    @NonNull
    double[] getCoefficients();

    /**
     * Sets the coefficients for this controller.
     * The order of the coefficients is expected to be the same as the return of the get method,
     * specified internally by the controller.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    void setCoefficients(@NonNull double[] coeffs);

    /**
     * Sets the coefficients for this controller.
     * The order of the coefficients is expected to be the same as the return of the get method,
     * specified internally by the controller.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    default void setCoefficients(@NonNull Double... coeffs) {
        setCoefficients(Arrays.stream(coeffs).mapToDouble(Double::doubleValue).toArray());
    }

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
