package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control;

import androidx.annotation.NonNull;

import java.util.Optional;
import java.util.function.DoubleBinaryOperator;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;

/**
 * Represents two recursively linked {@link SystemController} instances.
 *
 * @author Lucas Bubner, 2024
 * @see SystemController
 * @since 6.0.0
 */
public class CompositeController implements SystemController {
    /**
     * The first controller in the composite.
     */
    public final SystemController first;
    /**
     * The second controller in the composite.
     */
    public final SystemController second;
    private final DoubleBinaryOperator combiner;

    /**
     * Create a new composite controller that combines two controllers.
     *
     * @param first    the first controller
     * @param second   the second controller
     * @param combiner the function that combines the two controller outputs (example: {@code (a, b) -> a + b})
     */
    public CompositeController(@NonNull SystemController first, @NonNull SystemController second, @NonNull DoubleBinaryOperator combiner) {
        this.first = first;
        this.second = second;
        this.combiner = combiner;
    }

    @Override
    public double calculate(double process, double setpoint) {
        return combiner.applyAsDouble(first.calculate(process, setpoint), second.calculate(process, setpoint));
    }

    /**
     * Gets the coefficients of both controllers in this composite controller, concatenated as if they were one controller
     * with a length equal to the sum of the lengths of the two coefficient arrays.
     *
     * @return the concatenated coefficients of the two controllers
     */
    @NonNull
    @Override
    public double[] getCoefficients() {
        double[] firstCoeffs = first.getCoefficients();
        double[] secondCoeffs = second.getCoefficients();
        double[] combined = new double[firstCoeffs.length + secondCoeffs.length];
        System.arraycopy(firstCoeffs, 0, combined, 0, firstCoeffs.length);
        System.arraycopy(secondCoeffs, 0, combined, firstCoeffs.length, secondCoeffs.length);
        return combined;
    }

    /**
     * Sets the coefficients of both controllers in this composite controller.
     * <p>
     * The order of the coefficients is expected to be the same as the return of the get method of both controllers
     * that have been concatenated. For example, a PIDF controller followed by  3-arg system controller would have
     * the form {@code [kP, kI, kD, kF, k1, k2, k3]}.
     *
     * @param coeffs a list of coefficients to set this controller to
     */
    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != first.getCoefficients().length + second.getCoefficients().length) {
            throw new IllegalArgumentException("Invalid number of coefficients (expected " + (first.getCoefficients().length + second.getCoefficients().length) + ", got " + coeffs.length + ")");
        }
        double[] firstCoeffs = new double[first.getCoefficients().length];
        double[] secondCoeffs = new double[second.getCoefficients().length];
        System.arraycopy(coeffs, 0, firstCoeffs, 0, firstCoeffs.length);
        System.arraycopy(coeffs, firstCoeffs.length, secondCoeffs, 0, secondCoeffs.length);
        first.setCoefficients(firstCoeffs);
        second.setCoefficients(secondCoeffs);
    }

    /**
     * Resets both controllers in this composite controller.
     */
    @Override
    public void reset() {
        first.reset();
        second.reset();
    }

    /**
     * Gets the PIDF controller attached to this composite controller, if one exists.
     * If both controllers have PIDF controllers attached, an {@link IllegalArgumentException} will be thrown as
     * it is ambiguous which controller to use.
     *
     * @return the PIDF controller attached to this composite controller, if one exists
     */
    @Override
    public Optional<PIDFController> pidf() {
        if (first.pidf().isPresent() && second.pidf().isPresent()) {
            throw new IllegalArgumentException("Ambiguous PIDF controllers attached to this composed controller; cannot determine which one to use! " +
                    "Access the `first` and `second` fields of this CompositeController to get the individual controllers manually, or opt to using a different controller type that does not expose two PIDF controllers.");
        }
        if (first.pidf().isPresent()) {
            return Optional.of(first.pidf().get());
        }
        if (second.pidf().isPresent()) {
            return Optional.of(second.pidf().get());
        }
        return Optional.empty();
    }
}
