package au.edu.sa.mbhs.studentrobotics.bunyipslib.external;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Vector2d;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Reference;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Extended math utility functions.
 * <p>
 * This class is effectively a combination of the math found in WPILib's
 * <a href="https://github.com/wpilibsuite/allwpilib/blob/dc4c63568a2adbc2acfb5d6a420750236074b6aa/wpimath/src/main/java/edu/wpi/first/math/MathUtil.java">MathUtil</a>
 * and Unity's <a href="https://github.com/Unity-Technologies/UnityCsReference/blob/22a9cc4540dc5efa28ad9f02cd12b37b4b1a21c7/Runtime/Export/Math/Mathf.cs">Mathf</a> features,
 * adjusted to use WPIUnits and custom classes.
 * <p>
 * This class internally uses the Apache Commons {@link FastMath} methods.
 *
 * @see Math
 * @see FastMath
 * @see MathUtils
 * @since 1.0.0-pre
 */
public final class Mathf {
    private Mathf() {
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @return The rounded number, or 0 if the number is null
     */
    public static double round(@Nullable Number num, int thDigits) {
        return round(num, thDigits, -1);
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param num      The number to round
     * @param thDigits The number of decimal places to use after the decimal point
     * @param sigFigs  The number of significant figures to use
     * @return The rounded number, or 0 if the number is null, or 0 if the number is null
     */
    public static double round(@Nullable Number num, int thDigits, int sigFigs) {
        if (num == null)
            return 0;
        double n = num.doubleValue();
        if (thDigits == 0)
            return Math.round(n);
        BigDecimal bd = new BigDecimal(Double.toString(n));
        bd = bd.setScale(thDigits, RoundingMode.HALF_UP);
        if (sigFigs != -1)
            bd = bd.round(new MathContext(sigFigs, RoundingMode.HALF_UP));
        return bd.doubleValue();
    }

    /**
     * Checks if two values are approximately equal, such that floating point errors are accounted for.
     *
     * @param a the first number
     * @param b the second number
     * @return whether the two numbers are approximately equal by an epsilon of 1e-6
     */
    public static boolean approximatelyEquals(Number a, Number b) {
        return FastMath.abs(a.doubleValue() - b.doubleValue()) < 1.0e-6;
    }

    /**
     * Solve a quadratic for x, given coefficients a, b, and c.
     *
     * @param a Coefficient of x^2.
     * @param b Coefficient of x.
     * @param c Constant.
     * @return List of real roots.
     */
    public static List<Double> solveQuadratic(Number a, Number b, Number c) {
        double a_d = a.doubleValue(), b_d = b.doubleValue(), c_d = c.doubleValue();
        // Discriminant b^2-4ac
        double disc = b_d * b_d - 4 * a_d * c_d;
        if (approximatelyEquals(disc, 0)) {
            // One solution which is the same as setting x=0 for a linear function
            return Collections.singletonList(-b_d / (2 * a_d));
        }
        if (disc < 0) {
            // No real solutions
            return Collections.emptyList();
        }
        // Solutions at (-b \pm \sqrt{b^2-4ac}) / 2a
        return Arrays.asList(
                (-b_d + FastMath.sqrt(disc)) / 2 * a_d,
                (-b_d - FastMath.sqrt(disc)) / 2 * a_d
        );
    }

    /**
     * Normalizes the given angle to be within the range of [0, 2pi) radians [0, 360) degrees.
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    public static Measure<Angle> normaliseAngle(Measure<Angle> angle) {
        double ang = angle.in(Radians) % MathUtils.TWO_PI;
        return Radians.of((ang + MathUtils.TWO_PI) % MathUtils.TWO_PI);
    }

    /**
     * Normalizes the given radians to be within the range [0, 2pi).
     *
     * @param angrad The angle in radians.
     * @return The normalized radians in the range [0, 2pi)
     */
    public static double normaliseRadians(Number angrad) {
        double ang = angrad.doubleValue() % MathUtils.TWO_PI;
        return (ang + MathUtils.TWO_PI) % MathUtils.TWO_PI;
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(Number value, Number low, Number high) {
        return FastMath.max(low.doubleValue(), FastMath.min(value.doubleValue(), high.doubleValue()));
    }

    /**
     * Scale a number in the range of {@code x1} to {@code x2}, to the range of {@code y1} to {@code y2}.
     *
     * @param n  number to scale
     * @param x1 lower bound range of n
     * @param x2 upper bound range of n
     * @param y1 lower bound of scale
     * @param y2 upper bound of scale
     * @return a double scaled to a value between y1 and y2, inclusive
     */
    public static double scale(Number n, Number x1, Number x2, Number y1, Number y2) {
        double n_d = n.doubleValue(), x1_d = x1.doubleValue(), x2_d = x2.doubleValue(), y1_d = y1.doubleValue(), y2_d = y2.doubleValue();
        double m = (y1_d - y2_d) / (x1_d - x2_d);
        double c = y1_d - x1_d * (y1_d - y2_d) / (x1_d - x2_d);
        // y = mx + c
        return m * n.doubleValue() + c;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param value        Value to clip.
     * @param deadband     Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(Number value, Number deadband, Number maxMagnitude) {
        double value_d = value.doubleValue(), deadband_d = deadband.doubleValue(), maxMagnitude_d = maxMagnitude.doubleValue();
        if (FastMath.abs(value_d) > deadband_d) {
            if (maxMagnitude_d / deadband_d > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // round-off error.  Implementing the limiting behavior directly avoids
                // the problem.
                return value_d > 0.0 ? value_d - deadband_d : value_d + deadband_d;
            }
            if (value_d > 0.0) {
                // Map deadband to 0 and map max to max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                // x₁ = deadband
                // y₁ = 0
                // x₂ = max
                // y₂ = max
                //
                // y = (max - 0)/(max - deadband) (x - deadband) + 0
                // y = max/(max - deadband) (x - deadband)
                // y = max (x - deadband)/(max - deadband)
                return maxMagnitude_d * (value_d - deadband_d) / (maxMagnitude_d - deadband_d);
            } else {
                // Map -deadband to 0 and map -max to -max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                // x₁ = -deadband
                // y₁ = 0
                // x₂ = -max
                // y₂ = -max
                //
                // y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                // y = max/(max - deadband) (x + deadband)
                // y = max (x + deadband)/(max - deadband)
                return maxMagnitude_d * (value_d + deadband_d) / (maxMagnitude_d - deadband_d);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    Value to clip.
     * @param deadband Range around zero.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(Number value, Number deadband) {
        return applyDeadband(value, deadband, 1);
    }

    /**
     * Returns modulus of input.
     *
     * @param input        Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(Number input, Number minimumInput, Number maximumInput) {
        double input_d = input.doubleValue(), minimumInput_d = minimumInput.doubleValue(), maximumInput_d = maximumInput.doubleValue();
        double modulus = maximumInput_d - minimumInput_d;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input_d - minimumInput_d) / modulus);
        input_d -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input_d - maximumInput_d) / modulus);
        input_d -= numMin * modulus;

        return input_d;
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angle Angle to wrap.
     * @return The wrapped angle.
     */
    public static Measure<Angle> angleModulus(Measure<Angle> angle) {
        return Radians.of(inputModulus(angle.in(Radians), -FastMath.PI, FastMath.PI));
    }

    /**
     * Wraps radians to the range -pi to pi.
     *
     * @param angrad The angle in radians to wrap
     * @return Wrapped angle between -pi and pi
     */
    public static double radianModulus(Number angrad) {
        return inputModulus(angrad, -FastMath.PI, FastMath.PI);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(Number startValue, Number endValue, Number t) {
        return startValue.doubleValue() + (endValue.doubleValue() - startValue.doubleValue()) * clamp(t, 0, 1);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate.
     * @return The interpolated value.
     */
    public static double interpolateUnclamped(Number startValue, Number endValue, Number t) {
        return startValue.doubleValue() + (endValue.doubleValue() - startValue.doubleValue()) * t.doubleValue();
    }

    /**
     * Perform linear interpolation like {@link #interpolate(Number, Number, Number)}, but interpolates correctly when
     * they wrap around 1 revolution (360 degrees).
     *
     * @param a The start angle.
     * @param b The end angle.
     * @param t The interpolation parameter.
     * @return The interpolated value.
     */
    public static Measure<Angle> interpolateAngle(Measure<Angle> a, Measure<Angle> b, Number t) {
        double delta = repeat(b.in(Degrees) - a.in(Degrees), 360);
        if (delta > 180)
            delta -= 360;
        return Degrees.of(a.in(Degrees) + delta * clamp(t, 0, 1));
    }

    /**
     * Moves a value {@code current} towards {@code target}.
     *
     * @param current  The current value.
     * @param target   The value to move towards.
     * @param maxDelta The maximum change that should be applied to the value.
     * @return The new value.
     */
    public static double moveTowards(Number current, Number target, Number maxDelta) {
        double current_d = current.doubleValue(), target_d = target.doubleValue(), maxDelta_d = maxDelta.doubleValue();
        if (FastMath.abs(target_d - current_d) <= maxDelta_d)
            return target_d;
        return current_d + FastMath.signum(target_d - current_d) * maxDelta_d;
    }

    /**
     * Same as {@link #moveTowards(Number, Number, Number)}, but makes sure the values interpolate correctly when they
     * wrap around 1 revolution (360 degrees).
     *
     * @param current  The current angle.
     * @param target   The angle to move towards.
     * @param maxDelta The maximum change that should be applied to the value.
     * @return The new value.
     */
    public static Measure<Angle> moveTowardsAngle(Measure<Angle> current, Measure<Angle> target, Measure<Angle> maxDelta) {
        Measure<Angle> delta = deltaAngle(current, target);
        if (maxDelta.negate().lt(delta) && delta.lt(maxDelta))
            return target;
        return Degrees.of(moveTowards(current.in(Degrees), current.plus(delta).in(Degrees), maxDelta.in(Degrees)));
    }

    /**
     * Interpolates between min and max with smoothing at the limits.
     *
     * @param from The start value.
     * @param to   The end value.
     * @param t    The interpolation value between the two.
     * @return The smooth step value.
     */
    public static double smoothStep(Number from, Number to, Number t) {
        double newT = clamp(t, 0, 1);
        newT = -2.0F * newT * newT * newT + 3.0F * newT * newT;
        return to.doubleValue() * newT + from.doubleValue() * (1.0 - newT);
    }

    /**
     * Applies gamma correction to a given value within a specified range.
     *
     * @param value  the input value to be gamma corrected
     * @param absMax the maximum absolute value allowed in the input range
     * @param gamma  the gamma value for correction
     * @return the gamma corrected value
     */
    public static double gamma(Number value, Number absMax, Number gamma) {
        double value_d = value.doubleValue(), absMax_d = absMax.doubleValue(), gamma_d = gamma.doubleValue();
        boolean negative = value_d < 0.0;
        double absVal = FastMath.abs(value_d);
        if (absVal > absMax_d)
            return negative ? -absVal : absVal;
        double result = FastMath.pow(absVal / absMax_d, gamma_d) * absMax_d;
        return negative ? -result : result;
    }

    /**
     * Gradually changes a value towards a desired goal over time.
     *
     * @param current         The current position.
     * @param target          The position we want to be at.
     * @param currentVelocity The current velocity of the current position moving towards the target.
     *                        This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new position following the smooth damp. The new velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */
    public static double smoothDamp(Number current, Number target, @NonNull Reference<Double> currentVelocity, Measure<Time> smoothTime, Number maxVelocity, Measure<Time> deltaTime) {
        double current_d = current.doubleValue(), target_d = target.doubleValue(), maxVelocity_d = maxVelocity.doubleValue();

        double t = smoothTime.in(Seconds);
        double dt = deltaTime.in(Seconds);
        double omega = 2.0 / t;

        // Exponential decay function
        double x = omega * dt;
        double exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
        double delta = current_d - target_d;

        // Clamp maximum speed
        double maxDelta = maxVelocity_d * t;
        delta = clamp(delta, -maxDelta, maxDelta);

        // Calculate new velocity and output of the current position
        currentVelocity.ifNotPresent(() -> currentVelocity.set(0.0));
        double temp = (currentVelocity.require() + omega * delta) * dt;
        currentVelocity.set((currentVelocity.require() - omega * temp) * exp);
        double output = (current_d - delta) + (delta + temp) * exp;

        // Prevent overshooting
        if (target_d - current_d > 0.0 == output > target_d) {
            output = target_d;
            currentVelocity.set((output - target_d) / dt);
        }

        return output;
    }

    /**
     * Gradually changes an angle towards a desired goal over time.
     *
     * @param current         The current angle.
     * @param target          The angle we want to be at.
     * @param currentVelocity The current angular velocity of the current position moving towards the target.
     *                        This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new angle following the smooth damp. The new ang. velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */

    public static Measure<Angle> smoothDampAngle(Measure<Angle> current, Measure<Angle> target, Reference<Double> currentVelocity, Measure<Time> smoothTime, Number maxVelocity, Measure<Time> deltaTime) {
        double res = smoothDamp(current.in(Degrees), current.plus(deltaAngle(current, target)).in(Degrees), currentVelocity, smoothTime, maxVelocity, deltaTime);
        return Degrees.of(res);
    }

    /**
     * Loops the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to loop.
     * @param length The length of the loop.
     * @return The looped value.
     */
    public static double repeat(Number t, Number length) {
        double t_d = t.doubleValue(), length_d = length.doubleValue();
        return clamp(t_d - FastMath.floor(t_d / length_d) * length_d, 0.0f, length_d);
    }

    /**
     * PingPongs (bounces) the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to pingpong.
     * @param length The length of the pingpong.
     * @return The pingponged value.
     */
    public static double pingPong(Number t, Number length) {
        double length_d = length.doubleValue();
        double repeat = repeat(t, length_d * 2.0);
        return length_d - FastMath.abs(repeat - length_d);
    }

    /**
     * Calculates the shortest difference between two given angles.
     *
     * @param current The current angle.
     * @param target  The target angle.
     * @return The shortest difference between the two angles.
     */
    public static Measure<Angle> deltaAngle(Measure<Angle> current, Measure<Angle> target) {
        double delta = repeat(target.minus(current).in(Degrees), 360.0);
        if (delta > 180.0F)
            delta -= 360.0F;
        return Degrees.of(delta);
    }

    /**
     * Find the intersection between a line and a circle.
     *
     * @param p1     The first point of the line.
     * @param p2     The second point of the line.
     * @param center The position of the center of the circle.
     * @param radius The radius of the circle.
     * @return The intersection points.
     * @throws NoInterceptException If no intercepts are found.
     */
    public static Pair<Vector2d, Vector2d> lineCircleIntersection(Vector2d p1, Vector2d p2, Vector2d center, Number radius) throws NoInterceptException {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double a = dx * dx + dy * dy;
        double b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y));
        double c = center.x * center.x + center.y * center.y + p1.x * p1.x + p1.y * p1.y - 2 * (center.x * p1.x + center.y * p1.y) - radius.doubleValue() * radius.doubleValue();
        List<Double> solutions = solveQuadratic(a, b, c);
        if (solutions.isEmpty()) {
            throw new NoInterceptException();
        }
        double t1 = solutions.get(0);
        double t2 = solutions.size() == 2 ? solutions.get(1) : t1;
        return new Pair<>(new Vector2d(p1.x + t1 * dx, p1.y + t1 * dy), new Vector2d(p1.x + t2 * dx, p1.y + t2 * dy));
    }

    /**
     * Find the intersection between a line segment and a circle.
     *
     * @param p1     The first point of the line segment.
     * @param p2     The second point of the line segment.
     * @param center The position of the center of the circle.
     * @param radius The radius of the circle.
     * @return The intersection points.
     * @throws NoInterceptException If no intercepts are found.
     */
    public static Pair<Vector2d, Vector2d> lineSegmentCircleIntersection(Vector2d p1, Vector2d p2, Vector2d center, Number radius) throws NoInterceptException {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double a = dx * dx + dy * dy;
        double b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y));
        double c = center.x * center.x + center.y * center.y + p1.x * p1.x + p1.y * p1.y - 2 * (center.x * p1.x + center.y * p1.y) - radius.doubleValue() * radius.doubleValue();
        List<Double> solutions = solveQuadratic(a, b, c);
        if (solutions.isEmpty()) {
            throw new NoInterceptException();
        }
        double t1 = solutions.get(0);
        double t2 = solutions.size() == 2 ? solutions.get(1) : t1;
        if ((t1 < 0 || t1 > 1) && (t2 < 0 || t2 > 1)) {
            throw new NoInterceptException();
        }
        return new Pair<>(new Vector2d(p1.x + t1 * dx, p1.y + t1 * dy), new Vector2d(p1.x + t2 * dx, p1.y + t2 * dy));
    }

    /**
     * Find the intersection of two lines.
     *
     * @param p1 The first point of the first line.
     * @param p2 The second point of the first line.
     * @param p3 The first point of the second line.
     * @param p4 The second point of the second line.
     * @return The intersection point.
     * @throws NoInterceptException If no intercept is found.
     */
    public static Vector2d lineIntersection(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) throws NoInterceptException {
        double bx = p2.x - p1.x;
        double by = p2.y - p1.y;
        double dx = p4.x - p3.x;
        double dy = p4.y - p3.y;

        double bDotDPerp = bx * dy - by * dx;
        if (bDotDPerp == 0) {
            throw new NoInterceptException();
        }
        double cx = p3.x - p1.x;
        double cy = p3.y - p1.y;
        double t = (cx * dy - cy * dx) / bDotDPerp;

        return new Vector2d(p1.x + t * bx, p1.y + t * by);
    }

    /**
     * Find the intersection of two line segments.
     *
     * @param p1 The first point of the first line segment.
     * @param p2 The second point of the first line segment.
     * @param p3 The first point of the second line segment.
     * @param p4 The second point of the second line segment.
     * @return The intersection point.
     * @throws NoInterceptException If no intercept is found.
     */
    public static Vector2d lineSegmentIntersection(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) throws NoInterceptException {
        double bx = p2.x - p1.x;
        double by = p2.y - p1.y;
        double dx = p4.x - p3.x;
        double dy = p4.y - p3.y;

        double bDotDPerp = bx * dy - by * dx;
        if (bDotDPerp == 0) {
            throw new NoInterceptException();
        }
        double cx = p3.x - p1.x;
        double cy = p3.y - p1.y;
        double t = (cx * dy - cy * dx) / bDotDPerp;
        if (t < 0 || t > 1) {
            throw new NoInterceptException();
        }
        double u = (cx * by - cy * bx) / bDotDPerp;
        if (u < 0 || u > 1) {
            throw new NoInterceptException();
        }

        return new Vector2d(p1.x + t * bx, p1.y + t * by);
    }

    /**
     * Returns the next power of two that is equal to or larger than the specified value.
     *
     * @param value The value.
     * @return The next power of two.
     */
    public static int nextPowerOfTwo(int value) {
        int i = value;
        i -= 1;
        i |= i >> 16;
        i |= i >> 8;
        i |= i >> 4;
        i |= i >> 2;
        i |= i >> 1;
        return i + 1;
    }

    /**
     * Returns the closest power of two that is equal to or larger than the specified value.
     *
     * @param value The value.
     * @return The closest power of two.
     */
    public static int closestPowerOfTwo(int value) {
        int nextPower = nextPowerOfTwo(value);
        int prevPower = nextPower >> 1;
        if (value - prevPower < nextPower - value)
            return prevPower;
        else
            return nextPower;
    }

    /**
     * Returns whether the given value is a power of two.
     *
     * @param value The value.
     * @return Whether the value is a power of two.
     */
    public static boolean isPowerOfTwo(int value) {
        return (value & (value - 1)) == 0;
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    public static double inverseInterpolate(Number startValue, Number endValue, Number q) {
        double startValue_d = startValue.doubleValue();
        double totalRange = endValue.doubleValue() - startValue_d;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q.doubleValue() - startValue_d;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(Number expected, Number actual, Number tolerance) {
        double tolerance_d = tolerance.doubleValue();
        if (tolerance_d < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        return FastMath.abs(expected.doubleValue() - actual.doubleValue()) < tolerance_d;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance. Supports
     * continuous input for cases like absolute encoders.
     *
     * <p>Continuous input means that the min and max value are considered to be the same point, and
     * tolerances can be checked across them. A common example would be for absolute encoders: calling
     * isNear(2, 359, 5, 0, 360) returns true because 359 is 1 away from 360 (which is treated as the
     * same as 0) and 2 is 2 away from 0, adding up to an error of 3 degrees, which is within the
     * given tolerance of 5.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @param min       Smallest value before wrapping around to the largest value
     * @param max       Largest value before wrapping around to the smallest value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(Number expected, Number actual, Number tolerance, Number min, Number max) {
        double tolerance_d = tolerance.doubleValue();
        if (tolerance_d < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        // Max error is exactly halfway between the min and max
        double errorBound = (max.doubleValue() - min.doubleValue()) / 2.0;
        double error = inputModulus(expected.doubleValue() - actual.doubleValue(), -errorBound, errorBound);
        return FastMath.abs(error) < tolerance_d;
    }

    /**
     * Exception thrown if no intercept is found when using the intersection methods of this class.
     */
    public static class NoInterceptException extends RuntimeException {
        /**
         * Create a new NoInterceptException.
         */
        public NoInterceptException() {
            super("No intercept found");
        }
    }
}