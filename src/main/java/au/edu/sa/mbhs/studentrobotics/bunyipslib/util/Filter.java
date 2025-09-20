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
         * Higher values of R puts more trust in the model.
         */
        public double R;
        /**
         * Higher values of Q trusts the external sensor more.
         */
        public double Q;

        private double p = 1;
        private double x;
        private double lx;

        /**
         * Construct a new 1D Kalman filter.
         *
         * @param R higher values of R puts more trust in the model
         * @param Q higher values of Q trusts the sensor more
         */
        public Kalman(double R, double Q) {
            this.R = R;
            this.Q = Q;
        }

        /**
         * Calculate a new Kalman filter estimate.
         *
         * @param absModel the current absolute reading from the model system, will internally take a delta step of this model
         * @param sensor   the current reading from the sensor
         * @return Kalman gain compensated output
         */
        public double calculate(double absModel, double sensor) {
            // Take a delta here
            x += absModel - lx;
            update(sensor);
            lx = absModel;
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
            // Run calculate but don't accumulate lx
            x += incModel;
            update(sensor);
            return x;
        }

        // https://www.ctrlaltftc.com/advanced/the-kalman-filter
        private void update(double sensor) {
            // New variance projection
            double np = p + Q;
            // Converge new K gain
            double K = np / (np + R);
            // Apply K gain
            x += K * (sensor - x);
            p = (1 - K) * np;
        }

        /**
         * Resets the accumulated system state to zero. May need to be called in tandem with {@link #resetVariance()}.
         */
        public void reset() {
            x = 0;
        }

        /**
         * Sets the accumulated running value.
         *
         * @param val the new accumulated value (system state) this filter should continue from.
         */
        public void setAccumulatedValue(double val) {
            x = val;
        }

        /**
         * Reset the projected model variance to 1. May need to be called in tandem with {@link #reset()}.
         */
        public void resetVariance() {
            p = 1;
        }

        /**
         * Set an initial guess of the model's variance. Can cause the model to converge faster.
         */
        public void setVariance(double var) {
            p = var;
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
