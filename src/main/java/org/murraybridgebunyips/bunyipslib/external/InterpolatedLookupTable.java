package org.murraybridgebunyips.bunyipslib.external;

import androidx.annotation.NonNull;

import org.apache.commons.math3.exception.OutOfRangeException;
import org.apache.commons.math3.exception.util.LocalizedFormats;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Performs spline interpolation given a set of control points.
 * <p>
 * Utility from FTCLib
 * <a href="https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/util/InterpLUT.java">Source</a>
 *
 * @since 1.0.0-pre
 */
public class InterpolatedLookupTable {
    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private List<Double> mM = new ArrayList<>();
    private boolean built;

    /**
     * Creates a new instance of InterpolatedLookupTable to add values to.
     */
    public InterpolatedLookupTable() {
    }

    /**
     * Adds a control point to the lookup table.
     *
     * @param input  x
     * @param output y
     */
    public void add(double input, double output) {
        mX.add(input);
        mY.add(output);
    }

    /**
     * Creates a monotone cubic spline from a given set of control points.
     * <p>
     * The spline is guaranteed to pass through each control point exactly. Moreover, assuming the control points are
     * monotonic (Y is non-decreasing or non-increasing) then the interpolated values will also be monotonic.
     *
     * @throws IllegalArgumentException if the X or Y arrays are null, have different lengths or have fewer than 2 values.
     */
    //public static LUTWithInterpolator createLUT(List<Double> x, List<Double> y) {
    public void createLUT() {
        List<Double> x = mX;
        List<Double> y = mY;

        if (x == null || y == null || x.size() != y.size() || x.size() < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        int n = x.size();
        Double[] d = new Double[n - 1]; // could optimize this out
        Double[] m = new Double[n];

        // Compute slopes of secant lines between successive points.
        for (int i = 0; i < n - 1; i++) {
            double h = x.get(i + 1) - x.get(i);
            if (h <= 0.0f) {
                throw new IllegalArgumentException("The control points must all "
                        + "have strictly increasing X values.");
            }
            d[i] = (y.get(i + 1) - y.get(i)) / h;
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = (d[i - 1] + d[i]) * 0.5f;
        }
        m[n - 1] = d[n - 2];

        // Update the tangents to preserve monotonicity.
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0.0f) { // successive Y values are equal
                m[i] = 0.0;
                m[i + 1] = 0.0;
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9.0f) {
                    double t = 3.0f / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }
        mX = x;
        mY = y;
        mM = Arrays.asList(m);

        built = true;
    }

    /**
     * Check if a given input will be out of range by this LUT.
     *
     * @param input the value to check
     * @return 1 if the input is too large, -1 if it is too small, 0 if it is within bounds.
     */
    public int testOutOfRange(double input) {
        if (!built)
            throw new IllegalStateException("This InterpolatedLookupTable has not been built yet.");
        if (input <= mX.get(0)) {
            return -1;
        }
        if (input >= mX.get(mX.size() - 1)) {
            return 1;
        }
        return 0;
    }

    /**
     * @return the minimum domain value from this LUT.
     */
    public double getMin() {
        if (!built)
            throw new IllegalStateException("This InterpolatedLookupTable has not been built yet.");
        return get(mX.get(0) + 1.0e-6);
    }

    /**
     * @return the maximum domain value from this LUT.
     */
    public double getMax() {
        if (!built)
            throw new IllegalStateException("This InterpolatedLookupTable has not been built yet.");
        return get(mX.get(mX.size() - 1) - 1.0e-6);
    }

    /**
     * Interpolates the value of Y = f(X) for given X. Clamps X to the domain of the spline.
     *
     * @param input The X value.
     * @return The interpolated Y = f(X) value.
     * @throws OutOfRangeException if the input is outside of the domain available by this lookup table
     */
    public double get(double input) throws OutOfRangeException {
        if (!built)
            throw new IllegalStateException("This InterpolatedLookupTable has not been built yet.");

        // Handle the boundary cases.
        int n = mX.size();
        if (Double.isNaN(input)) {
            return input;
        }
        if (input <= mX.get(0)) {
            throw new OutOfRangeException(LocalizedFormats.NUMBER_TOO_SMALL_BOUND_EXCLUDED, input, mX.get(0), 0);
        }
        if (input >= mX.get(n - 1)) {
            throw new OutOfRangeException(LocalizedFormats.NUMBER_TOO_LARGE_BOUND_EXCLUDED, input, mX.get(n - 1), 0);
        }

        // Find the index 'i' of the last point with smaller X.
        // We know this will be within the spline due to the boundary tests.
        int i = 0;
        while (input >= mX.get(i + 1)) {
            i += 1;
            if (input == mX.get(i)) {
                return mY.get(i);
            }
        }

        // Perform cubic Hermite spline interpolation.
        double h = mX.get(i + 1) - mX.get(i);
        double t = (input - mX.get(i)) / h;
        return (mY.get(i) * (1 + 2 * t) + h * mM.get(i) * t) * (1 - t) * (1 - t)
                + (mY.get(i + 1) * (3 - 2 * t) + h * mM.get(i + 1) * (t - 1)) * t * t;
    }

    // For debugging.
    @NonNull
    @Override
    public String toString() {
        if (!built)
            throw new IllegalStateException("This InterpolatedLookupTable has not been built yet.");

        StringBuilder str = new StringBuilder();
        int n = mX.size();
        str.append("[");
        for (int i = 0; i < n; i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append("(").append(mX.get(i));
            str.append(", ").append(mY.get(i));
            str.append(": ").append(mM.get(i)).append(")");
        }
        str.append("]");
        return str.toString();
    }

}