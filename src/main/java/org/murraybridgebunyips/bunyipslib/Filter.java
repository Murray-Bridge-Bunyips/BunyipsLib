package org.murraybridgebunyips.bunyipslib;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NumberIsTooLargeException;
import org.apache.commons.math3.exception.NumberIsTooSmallException;
import org.murraybridgebunyips.bunyipslib.external.units.UnaryFunction;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

/**
 * A collection of data filters for smoothing out sensor data.
 *
 * @author Lucas Bubner, 2024
 */
public final class Filter {
    private Filter() {
        // No instances
    }

    /**
     * A <a href="https://en.wikipedia.org/wiki/Low-pass_filter">Low-pass filter</a> that uses a single gain to
     * measure filter response time, cleaning up noisy sensor inputs.
     */
    public static class LowPass implements UnaryFunction {
        private final double gain;
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
                throw new NumberIsTooSmallException(gain, 0, false);
            }
            if (gain >= 1) {
                throw new NumberIsTooLargeException(gain, 1, false);
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
         * @param inputs array of input suppliers, such as sensor inputs
         * @param weights mapping of parametric [0,1] multiplicative weights to apply to the sensor inputs,
         *                an example is if you have 2 inputs, and you trust them equally, setting both weights
         *                to 0.5 will make them equally important.
         */
        public WeightedFusion(DoubleSupplier[] inputs, double[] weights) {
            if (inputs.length != weights.length)
                throw new DimensionMismatchException(weights.length, inputs.length);
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
