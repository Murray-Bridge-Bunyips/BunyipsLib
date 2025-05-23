package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;

/**
 * A collection of data filters for smoothing out and fusing sensor data.
 *
 * @author Lucas Bubner, 2024
 * @since 4.1.0
 */
public final class Filter {
    private Filter() {
        throw new AssertionError("This is a utility class");
    }

    /**
     * A <a href="https://en.wikipedia.org/wiki/Low-pass_filter">Low-pass filter</a> that uses a single gain to
     * measure filter response time, cleaning up noisy sensor inputs.
     */
    public static class LowPass implements UnaryFunction {
        /**
         * Gain used in (0, 1), higher values of the gain are smoother but have more phase lag,
         */
        public final double gain;
        private boolean hasInit;
        private double lastValue;

        /**
         * Construct a new Low-pass filter with a gain in the domain (0, 1) exclusive.
         *
         * @param gain gain in (0, 1),
         *             higher values of the gain are smoother but have more phase lag,
         *             while low values of the gain allow more noise but will respond
         *             faster to quick changes in the measured state.
         */
        public LowPass(double gain) {
            if (gain <= 0) {
                throw new IllegalArgumentException("gain cannot be 0 or negative");
            }
            if (gain >= 1) {
                throw new IllegalArgumentException("gain cannot be equal to or greater than 1");
            }
            this.gain = gain;
        }

        /**
         * Calculate a Low-pass filter estimate.
         *
         * @param input the input value to the low pass filter
         * @return the filtered output
         */
        @Override
        public double apply(double input) {
            if (!hasInit) {
                // Pass through the first input to this filter to avoid a jumped response
                lastValue = input;
                hasInit = true;
            } else {
                lastValue = gain * lastValue + (1 - gain) * input;
            }
            return lastValue;
        }
    }

    /**
     * A basic 1D <a href="https://en.wikipedia.org/wiki/Kalman_filter">Kalman filter</a> that estimates state
     * from a model and sensor over time.
     */
    public static class Kalman {
        /**
         * Number of iterations to take during construction to converge the K gain.
         */
        public static int K_CONVERGE = 2000;

        private double kGain;
        private double x, lm;

        /**
         * Construct a new 1D Kalman filter. K gain is calculated at construction.
         *
         * @param R higher values of R put more trust in the model
         * @param Q higher values of Q trusts the sensor more
         */
        public Kalman(double R, double Q) {
            // We should converge K and P to calculate the gain that will be used for the duration of this filter
            kGain = 0;
            double p = 1;
            for (int i = 0; i < K_CONVERGE; i++) {
                // p_t=p_{t-1}+q
                // where p_{t-1} is supplied from the previous loop
                p += Q;
                // k_t=\frac{p_t}{p_t+r}
                kGain = p / (p + R);
                // Update uncertainty p_t=(1-K)p_t
                p = (1 - kGain) * p;
            }
        }

        /**
         * Calculate a new Kalman filter estimate.
         *
         * @param absModel the current absolute reading from the model system, will internally take a delta step of this model
         * @param sensor   the current reading from the sensor
         * @return Kalman gain compensated output
         */
        public double calculate(double absModel, double sensor) {
            // Since we're only dealing with scalars, we can make a lot of simplifications
            // Therefore filtered x_t=x_{t-1}+K(z_t-x_t)
            x += absModel - lm;
            x += kGain * (sensor - x);
            lm = absModel;
            return x;
        }

        /**
         * Calculate a new Kalman filter estimate.
         *
         * @param incModel the current model from the model system, supplied as an increment or delta
         * @param sensor   the current reading from the sensor
         * @return Kalman gain compensated output
         */
        public double calculateFromDelta(double incModel, double sensor) {
            // Run calculate but don't accumulate lm
            x += incModel;
            x += kGain * (sensor - x);
            return x;
        }

        /**
         * Reset accumulated value to zero.
         */
        public void reset() {
            x = 0;
        }

        /**
         * Sets the accumulated value.
         *
         * @param val the accumulated value to respect.
         */
        public void setAccumulatedValue(double val) {
            x = val;
        }
    }

    /**
     * Fuses weighted data inputs together to return a weighted fused average.
     */
    public static class WeightedFusion implements DoubleSupplier {
        private final DoubleSupplier[] inputs;
        private final double[] weights;

        /**
         * Construct a filter that fuses two weighted inputs together.
         * <p>
         * Note: {@code inputs.length == weights.length}
         *
         * @param inputs  array of input suppliers, such as sensor inputs
         * @param weights mapping of parametric [0,1] multiplicative weights to apply to the sensor inputs,
         *                an example is if you have 2 inputs, and you trust them equally, setting both weights
         *                to 0.5 will make them equally important.
         */
        public WeightedFusion(@NonNull DoubleSupplier[] inputs, @NonNull double[] weights) {
            if (inputs.length != weights.length)
                throw new IllegalArgumentException("dimension mismatch, must be square: " + weights.length + "x" + inputs.length);
            this.inputs = inputs;
            this.weights = weights;
        }

        /**
         * Calculate a fused weighted average of the inputs.
         *
         * @return weighted average of the supplied inputs and weights
         */
        @Override
        public double getAsDouble() {
            double weighedSum = 0;
            for (int i = 0; i < inputs.length; i++) {
                weighedSum += weights[i] * inputs[i].getAsDouble();
            }
            return weighedSum / Arrays.stream(weights).sum();
        }
    }
}
