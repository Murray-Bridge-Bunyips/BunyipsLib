// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units;

import androidx.annotation.NonNull;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

/**
 * A function that accepts a single {@code double} and returns a {@code double} result. This is used
 * to represent arbitrary mapping functions for converting units to and from a base unit
 * representation. Temperature units, in particular, typically have an offset from a value in Kelvin
 * and may have a multiplication factor added in, which means that units cannot always be
 * represented as simple ratios of their base units.
 *
 * @since 1.0.0-pre
 */
@FunctionalInterface
public interface UnaryFunction extends DoubleUnaryOperator {
    /**
     * The identity function that simply returns the input value.
     */
    UnaryFunction IDENTITY = x -> x;
    /**
     * A function that squares the input value and keeps the sign.
     */
    UnaryFunction SQUARE_KEEP_SIGN = x -> Math.copySign(x * x, x);
    /**
     * A function that squares the input value.
     */
    UnaryFunction SQUARE = x -> x * x;
    /**
     * A function that cubes the input value and keeps the sign.
     */
    UnaryFunction CUBE_KEEP_SIGN = x -> Math.copySign(x * x * x, x);
    /**
     * A function that cubes the input value.
     */
    UnaryFunction CUBE = x -> x * x * x;
    /**
     * A function that negates the input value.
     */
    UnaryFunction NEGATE = x -> -x;

    /**
     * Applies this function to the input value and returns the result.
     *
     * @param input the input value to the function
     * @return the result
     */
    double apply(double input);

    /**
     * Applies this function to the input value and returns the result.
     *
     * @param operand the input value to the function
     * @return the result
     */
    @Override
    default double applyAsDouble(double operand) {
        return apply(operand);
    }

    /**
     * Constructs a new function that first calls this function, then passes the result to another as
     * input.
     *
     * <pre>
     * f = x -&gt; x + 1 // f(x) = x + 1
     * g = x -&gt; 2 * x // g(x) = 2x
     *
     * h = f.pipeTo(g) // h(x) = g(f(x))
     * </pre>
     *
     * @param next the next operation to pipe to
     * @return the composite function g(f(x))
     */
    @NonNull
    default UnaryFunction pipeTo(@NonNull UnaryFunction next) {
        Objects.requireNonNull(next, "The next operation in the chain must be provided");

        return x -> next.apply(apply(x));
    }

    /**
     * Creates a composite function h(x) such that h(x) = f(x) * g(x).
     *
     * @param multiplier the function to multiply this one by
     * @return the composite function f(x) * g(x)
     */
    @NonNull
    default UnaryFunction mult(@NonNull UnaryFunction multiplier) {
        Objects.requireNonNull(multiplier, "A multiplier function must be provided");

        return x -> apply(x) * multiplier.apply(x);
    }

    /**
     * Creates a composite function h(x) such that h(x) = k * f(x).
     *
     * @param multiplier the constant value to multiply this function's results by
     * @return the composite function k * f(x)
     */
    @NonNull
    default UnaryFunction mult(double multiplier) {
        return x -> apply(x) * multiplier;
    }

    /**
     * Creates a composite function h(x) such that h(x) = f(x) / g(x).
     *
     * @param divisor the function to divide this one by
     * @return the composite function f(x) / g(x)
     */
    @NonNull
    default UnaryFunction div(@NonNull UnaryFunction divisor) {
        Objects.requireNonNull(divisor, "A divisor function must be provided");

        return x -> {
            double numerator = apply(x);

            // fast-track to avoid another function call
            // avoids returning NaN if divisor is also zero
            if (numerator == 0) {
                return 0;
            }

            double div = divisor.apply(x);
            return numerator / div; // NOTE: returns +Infinity or -Infinity if div is zero
        };
    }

    /**
     * Creates a composite function h(x) such that h(x) = 1/k * f(x).
     *
     * @param divisor the constant value to divide this function's results by
     * @return the composite function 1/k * f(x)
     */
    @NonNull
    default UnaryFunction div(double divisor) {
        return x -> apply(x) / divisor;
    }

    /**
     * Creates a composite function h(x) such that h(x) = f(x) ^ g(x).
     *
     * @param exponent the function to exponentiate this function's results by
     * @return the composite function f(x) ^ g(x)
     */
    @NonNull
    default UnaryFunction exp(@NonNull UnaryFunction exponent) {
        Objects.requireNonNull(exponent, "An exponent function must be provided");

        return x -> Math.pow(apply(x), exponent.apply(x));
    }

    /**
     * Creates a composite function h(x) such that h(x) = f(x) ^ k.
     *
     * @param exponent the constant value to exponentiate this function's results by
     * @return the composite function f(x) ^ k
     */
    @NonNull
    default UnaryFunction exp(double exponent) {
        return x -> Math.pow(apply(x), exponent);
    }
}
