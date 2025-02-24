package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Filter;

/**
 * Performs numerical differentiation of a value with respect to time (sec).
 * Supports a low-pass filter which can be used to smoothen out results from the derivative.
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
public class Diff implements UnaryFunction {
    /**
     * Default low-pass gain to use when applying the derivative.
     */
    public static double DEFAULT_LOW_PASS_GAIN = 0.95;

    private Filter.LowPass filter = new Filter.LowPass(DEFAULT_LOW_PASS_GAIN);
    private double lastTimestamp = -1;
    private double lastInput, derivative;

    /**
     * Set a new Low Pass filter gain for use with derivative readings.
     *
     * @param gain the gain in the interval (0, 1) exclusive; default of 0.95
     */
    public void setLowPassGain(double gain) {
        if (gain == filter.gain) return;
        filter = new Filter.LowPass(gain);
    }

    @Override
    public double apply(double input) {
        double timestamp = System.nanoTime() / 1.0E9;
        if (lastTimestamp == -1)
            lastTimestamp = timestamp;
        double deltaTime = timestamp - lastTimestamp;
        // Reject values too small for dt due to division
        if (deltaTime > 1.0E-3) {
            derivative = filter.apply((input - lastInput) / deltaTime);
            lastInput = input;
            lastTimestamp = timestamp;
        }
        // Will be initially 0 until deltaTime period passes
        return derivative;
    }

    /**
     * @return the last result returned from {@link #apply(double)}.
     */
    public double getLast() {
        return derivative;
    }
}
