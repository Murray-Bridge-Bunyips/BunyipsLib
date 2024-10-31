package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleBinaryOperator;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;

/**
 * Generic interface that represents a controller algorithm to move a system from one state to another.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
@FunctionalInterface
public interface SystemController {
    /**
     * A null controller that does nothing.
     */
    SystemController NULL = (process, setpoint) -> 0;

    /**
     * Calculate the next output of this control algorithm.
     *
     * @param process  the current value
     * @param setpoint the expected value
     * @return process variable that should be applied to the system
     */
    double calculate(double process, double setpoint);

    /**
     * Gets the current coefficients for this controller.
     * The return of this method is expected to be in the same order and length as the set method.
     * On anonymous controllers, this method may return an empty array.
     *
     * @return an array of coefficients for this controller
     */
    @NonNull
    default double[] getCoefficients() {
        return new double[0];
    }

    /**
     * Sets the coefficients for this controller.
     * The order of the coefficients is expected to be the same as the return of the get method,
     * specified internally by the controller.
     * On anonymous controllers, this method may be a no-op.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    default void setCoefficients(@NonNull double[] coeffs) {
        // Default impl, no-op
    }

    /**
     * Sets the coefficients for this controller.
     * The order of the coefficients is expected to be the same as the return of the get method,
     * specified internally by the controller.
     * On anonymous controllers, this method may be a no-op.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    default void setCoefficients(@NonNull Double... coeffs) {
        setCoefficients(Arrays.stream(coeffs).mapToDouble(Double::doubleValue).toArray());
    }

    /**
     * Composes this controller with another controller, returning a new controller that is the composition of the two.
     * <p>
     * The specified BiConsumer that returns a Double is expected to be a function that indicates how the two controllers
     * should be combined. (e.g. addition, multiplication, etc).
     *
     * @param other    the other controller to compose with
     * @param combiner the function that combines the two controllers
     * @return a new controller that is the composition of the two
     */
    default CompositeController compose(SystemController other, DoubleBinaryOperator combiner) {
        return new CompositeController(this, other, combiner);
    }

    /**
     * Reset this controller back to an un-accumulated state, if applicable.
     * This method is not required to be implemented, and may be a no-op.
     */
    default void reset() {
        // Default impl, no-op
    }

    /**
     * Get the PIDF controller associated with this controller, if applicable.
     *
     * @return the PIDF controller associated with this controller, if applicable
     */
    default Optional<PIDFController> pidf() {
        // Default impl, no PIDF controller
        return Optional.empty();
    }
}
