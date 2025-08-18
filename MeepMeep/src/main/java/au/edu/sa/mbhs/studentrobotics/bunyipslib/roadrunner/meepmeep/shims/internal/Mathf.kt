package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Degrees
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Radians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Seconds
import com.acmerobotics.roadrunner.Vector2d
import dev.frozenmilk.util.cell.Cell
import java.lang.Math.PI
import java.math.BigDecimal
import java.math.MathContext
import java.math.RoundingMode
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.roundToLong
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Extended math utility functions.
 *
 * This class is effectively a combination of the math found in WPILib's
 * [MathUtil](https://github.com/wpilibsuite/allwpilib/blob/dc4c63568a2adbc2acfb5d6a420750236074b6aa/wpimath/src/main/java/edu/wpi/first/math/MathUtil.java)
 * and Unity's [Mathf](https://github.com/Unity-Technologies/UnityCsReference/blob/22a9cc4540dc5efa28ad9f02cd12b37b4b1a21c7/Runtime/Export/Math/Mathf.cs) features,
 * adjusted to use WPIUnits and custom classes.
 *
 * @see Math
 * @since 1.0.0-pre
 */
object Mathf {
    /**
     * Round a number to a certain number of decimal points.
     *
     * @param thDigits The number of decimal places to use after the decimal point
     * @param sigFigs  The number of significant figures to use
     * @return The rounded number, or 0 if the number is null, or 0 if the number is null
     */
    @JvmOverloads
    @JvmStatic
    fun Number?.round(thDigits: Int, sigFigs: Int = -1): Double {
        if (this == null || this.toDouble().isNaN()) return 0.0
        val n = this.toDouble()
        if (thDigits == 0) return n.roundToLong().toDouble()
        var bd = BigDecimal(n.toString())
        bd = bd.setScale(thDigits, RoundingMode.HALF_UP)
        if (sigFigs != -1) bd = bd.round(MathContext(sigFigs, RoundingMode.HALF_UP))
        return bd.toDouble()
    }

    /**
     * Round a number to a certain number of decimal points.
     *
     * @param thDigits The number of decimal places to use after the decimal point
     * @return The rounded number, or 0 if the number is null, or 0 if the number is null
     */
    @JvmStatic
    @JvmName("roundInfix")
    infix fun Number?.round(thDigits: Int): Double {
        return round(thDigits, -1)
    }

    /**
     * Checks if two values are approximately equal, such that floating point errors are accounted for.
     *
     * @param other the other number
     * @return whether the two numbers are approximately equal by an epsilon of 1e-6
     */
    @JvmStatic
    infix fun Number.approx(other: Number): Boolean {
        return abs(this.toDouble() - other.toDouble()) < 1.0e-6
    }

    /**
     * Solve a quadratic for x, given coefficients a, b, and c.
     *
     * @param a Coefficient of x^2.
     * @param b Coefficient of x.
     * @param c Constant.
     * @return List of real roots.
     */
    @JvmStatic
    fun solveQuadratic(a: Number, b: Number, c: Number): List<Double> {
        val aD = a.toDouble()
        val bD = b.toDouble()
        val cD = c.toDouble()
        // Discriminant b^2-4ac
        val disc = bD * bD - 4 * aD * cD
        if (disc approx 0) {
            // One solution which is the same as setting x=0 for a linear function
            return listOf(-bD / (2 * aD))
        }
        if (disc < 0) {
            // No real solutions
            return emptyList()
        }
        // Solutions at (-b \pm \sqrt{b^2-4ac}) / 2a
        return listOf(
            (-bD + sqrt(disc)) / (2 * aD),
            (-bD - sqrt(disc)) / (2 * aD)
        )
    }

    /**
     * Normalizes the given angle to be within the range of [0, 2π) radians or [0, 360) degrees.
     *
     * @return The normalized angle.
     */
    @JvmStatic
    fun Measure<Angle>.wrap(): Measure<Angle> {
        val ang = (this to Radians) % TWO_PI
        return Degrees.of(((ang + TWO_PI) % TWO_PI).radToDeg())
    }

    /**
     * Normalizes the given radians to be within the range [0, 2π).
     *
     * @return The normalized radians in the range [0, 2π)
     */
    @JvmStatic
    fun Number.wrapRadians(): Double {
        val ang = this.toDouble() % TWO_PI
        return (ang + TWO_PI) % TWO_PI
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    @JvmStatic
    fun Number.clamp(low: Number, high: Number): Double {
        return max(low.toDouble(), min(this.toDouble(), high.toDouble()))
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param range The range to which to clamp value.
     * @return The clamped value.
     */
    @JvmStatic
    infix fun Number.clamp(range: ClosedFloatingPointRange<Double>): Double {
        return max(range.start, min(this.toDouble(), range.endInclusive))
    }

    /**
     * Scale a number in the range of `x1` to `x2`, to the range of `y1` to `y2`.
     *
     * Inputs outside the bounds will be mapped correspondingly to outputs inside the output bounds.
     * Inputs equal to [x1] will be mapped to [y1], and inputs equal to [x2] will be mapped to [y2].
     *
     * @param x1 lower bound range of n
     * @param x2 upper bound range of n
     * @param y1 lower bound of scale
     * @param y2 upper bound of scale
     * @return a double scaled to a value between y1 and y2, inclusive
     */
    @JvmStatic
    fun Number.scale(x1: Number, x2: Number, y1: Number, y2: Number): Double {
        val x1D = x1.toDouble()
        val x2D = x2.toDouble()
        val y1D = y1.toDouble()
        val y2D = y2.toDouble()
        val m = (y1D - y2D) / (x1D - x2D)
        val c = y1D - x1D * (y1D - y2D) / (x1D - x2D)
        // y = mx + c
        return m * this.toDouble() + c
    }

    /**
     * Scale a number in the range of `x1` to `x2`, to the range of `y1` to `y2`.
     *
     * Inputs outside the bounds will be mapped correspondingly to outputs inside the output bounds.
     * Inputs equal to [x1x2] will be mapped to [y1y2].
     *
     * @param x1x2 range of n
     * @param y1y2 range of scale
     * @return a double scaled to a value between y1 and y2, inclusive
     */
    @JvmStatic
    fun Number.scale(x1x2: ClosedFloatingPointRange<Double>, y1y2: ClosedFloatingPointRange<Double>): Double {
        return this.scale(x1x2.start, x1x2.endInclusive, y1y2.start, y1y2.endInclusive)
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param deadband     Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */
    @JvmStatic
    @JvmOverloads
    fun Number.applyDeadband(deadband: Number, maxMagnitude: Number = 1): Double {
        val valueD = this.toDouble()
        val deadbandD = deadband.toDouble()
        val maxMagnitudeD = maxMagnitude.toDouble()
        if (abs(valueD) > deadbandD) {
            if (maxMagnitudeD / deadbandD > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // round-off error.  Implementing the limiting behavior directly avoids
                // the problem.
                return if (valueD > 0.0) valueD - deadbandD else valueD + deadbandD
            }
            return if (valueD > 0.0) {
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
                maxMagnitudeD * (valueD - deadbandD) / (maxMagnitudeD - deadbandD)
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
                maxMagnitudeD * (valueD + deadbandD) / (maxMagnitudeD - deadbandD)
            }
        } else {
            return 0.0
        }
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     * Maximal magnitude is 1 for this infix overload.
     *
     * @param deadband     Range around zero.
     * @return The value after the deadband is applied.
     */
    @JvmStatic
    @JvmName("applyDeadbandInfix")
    infix fun Number.applyDeadband(deadband: Number): Double {
        return this.applyDeadband(deadband, 1)
    }

    /**
     * Returns modulus of input.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    @JvmStatic
    fun Number.wrap(minimumInput: Number, maximumInput: Number): Double {
        var inputD = this.toDouble()
        val minimumInputD = minimumInput.toDouble()
        val maximumInputD = maximumInput.toDouble()
        val modulus = maximumInputD - minimumInputD

        // Wrap input if it's above the maximum input
        val numMax = ((inputD - minimumInputD) / modulus).toInt()
        inputD -= numMax * modulus

        // Wrap input if it's below the minimum input
        val numMin = ((inputD - maximumInputD) / modulus).toInt()
        inputD -= numMin * modulus

        return inputD
    }

    /**
     * Returns modulus of input.
     *
     * @param range The range of the input.
     * @return The wrapped value.
     */
    @JvmStatic
    infix fun Number.wrap(range: ClosedFloatingPointRange<Double>): Double {
        return this.wrap(range.start, range.endInclusive)
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @return The wrapped angle.
     */
    @JvmStatic
    fun Measure<Angle>.wrapDelta(): Measure<Angle> {
        return Degrees.of(this to Degrees wrap (-180.0..180.0))
    }

    /**
     * Wraps radians to the range -pi to pi.
     *
     * @return Wrapped angle between -pi and pi
     */
    @JvmStatic
    fun Number.wrapDeltaRadians(): Double {
        return this wrap (-PI..PI)
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    @JvmStatic
    fun lerp(startValue: Number, endValue: Number, t: Number): Double {
        return startValue.toDouble() + (endValue.toDouble() - startValue.toDouble()) * (t clamp (0.0..1.0))
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    @JvmStatic
    infix fun ClosedFloatingPointRange<Double>.lerp(t: Number): Double {
        return lerp(this.start, this.endInclusive, t)
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate.
     * @return The interpolated value.
     */
    @JvmStatic
    fun lerpUnclamped(startValue: Number, endValue: Number, t: Number): Double {
        return startValue.toDouble() + (endValue.toDouble() - startValue.toDouble()) * t.toDouble()
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param t          How far between the two values to interpolate.
     * @return The interpolated value.
     */
    @JvmStatic
    infix fun ClosedFloatingPointRange<Double>.lerpUnclamped(t: Number): Double {
        return lerpUnclamped(this.start, this.endInclusive, t)
    }

    /**
     * Perform linear interpolation like [lerp], but interpolates correctly when
     * they wrap around 1 revolution (360 degrees).
     *
     * @param a The start angle.
     * @param b The end angle.
     * @param t The interpolation parameter.
     * @return The interpolated value.
     */
    @JvmStatic
    fun lerp(a: Measure<Angle>, b: Measure<Angle>, t: Number): Measure<Angle> {
        var delta = ((b to Degrees) - (a to Degrees)) repeat 360
        if (delta > 180) delta -= 360.0
        return Degrees.of((a to Degrees) + delta * (t clamp (0.0..1.0)))
    }

    /**
     * Perform linear interpolation like [lerp], but interpolates correctly when
     * they wrap around 1 revolution (360 degrees).
     *
     * @param t The interpolation parameter.
     * @return The interpolated value.
     */
    @JvmStatic
    infix fun Pair<Measure<Angle>, Measure<Angle>>.lerp(t: Number): Measure<Angle> {
        return lerp(first, second, t)
    }

    /**
     * Moves a current value towards `target`.
     *
     * @param target   The value to move towards.
     * @param maxDelta The maximum change that should be applied to the value. Negative values will push the current value away from the target for potential convergence of wrapped numbers.
     * @return The new value.
     */
    @JvmStatic
    fun Number.moveTowards(target: Number, maxDelta: Number): Double {
        val currentD = this.toDouble()
        val targetD = target.toDouble()
        val maxDeltaD = maxDelta.toDouble()
        if (abs(targetD - currentD) <= abs(maxDeltaD)) return targetD
        return currentD + sign(targetD - currentD) * maxDeltaD
    }

    /**
     * Same as [moveTowards], but makes sure the values interpolate correctly when they
     * wrap around 1 revolution (360 degrees).
     *
     * @param target   The angle to move towards.
     * @param maxDelta The maximum change that should be applied to the value. Negative values will push the current value away from the target for potential convergence of wrapped numbers.
     * @return The new value.
     */
    @JvmStatic
    fun Measure<Angle>.moveTowards(target: Measure<Angle>, maxDelta: Measure<Angle>): Measure<Angle> {
        val delta = this diff target
        if (maxDelta.negate().lt(delta) && delta.lt(maxDelta)) return target
        return Degrees.of((this to Degrees).moveTowards((this + delta) to Degrees, maxDelta to Degrees))
    }

    /**
     * Interpolates between min and max with smoothing at the limits.
     *
     * @param from The start value.
     * @param to   The end value.
     * @param t    The interpolation value between the two.
     * @return The smooth step value.
     */
    @JvmStatic
    fun smoothStep(from: Number, to: Number, t: Number): Double {
        var newT = t clamp (0.0..1.0)
        newT = -2.0f * newT * newT * newT + 3.0f * newT * newT
        return to.toDouble() * newT + from.toDouble() * (1.0 - newT)
    }

    /**
     * Interpolates between min and max with smoothing at the limits.
     *
     * @param t    The interpolation value between the two.
     * @return The smooth step value.
     */
    @JvmStatic
    fun ClosedFloatingPointRange<Double>.smoothStep(t: Number): Double {
        var newT = t clamp (0.0..1.0)
        newT = -2.0f * newT * newT * newT + 3.0f * newT * newT
        return this.start * newT + this.endInclusive * (1.0 - newT)
    }

    /**
     * Applies gamma correction to a given value within a specified range.
     *
     * @param absMax the maximum absolute value allowed in the input range
     * @param gamma  the gamma value for correction
     * @return the gamma corrected value
     */
    @JvmStatic
    fun Number.gamma(absMax: Number, gamma: Number): Double {
        val valueD = this.toDouble()
        val absMaxD = absMax.toDouble()
        val gammaD = gamma.toDouble()
        val negative = valueD < 0.0
        val absVal = abs(valueD)
        if (absVal > absMaxD) return if (negative) -absVal else absVal
        val result = (absVal / absMaxD).pow(gammaD) * absMaxD
        return if (negative) -result else result
    }

    /**
     * Gradually changes a value towards a desired goal over time.
     *
     * @param target          The position we want to be at.
     * @param currentVelocity The current velocity of the current position moving towards the target.
     * This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new position following the smooth damp. The new velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */
    @JvmStatic
    fun Number.smoothDamp(
        target: Number,
        currentVelocity: Cell<Double>,
        smoothTime: Measure<Time>,
        maxVelocity: Number,
        deltaTime: Measure<Time>
    ): Double {
        val currentD = this.toDouble()
        val targetD = target.toDouble()
        val maxVelocityD = maxVelocity.toDouble()

        val t = smoothTime to Seconds
        val dt = deltaTime to Seconds
        val omega = 2.0 / t

        // Exponential decay function
        val x = omega * dt
        val exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
        var delta = currentD - targetD

        // Clamp maximum speed
        val maxDelta = maxVelocityD * t
        delta = delta clamp (-maxDelta..maxDelta)

        // Calculate new velocity and output of the current position
        val temp = (currentVelocity.get() + omega * delta) * dt
        currentVelocity.accept((currentVelocity.get() - omega * temp) * exp)
        var output = (currentD - delta) + (delta + temp) * exp

        // Prevent overshooting
        if (targetD - currentD > 0.0 == output > targetD) {
            output = targetD
            currentVelocity.accept((output - targetD) / dt)
        }

        return output
    }

    /**
     * Gradually changes an angle towards a desired goal over time.
     *
     * @param target          The angle we want to be at.
     * @param currentVelocity The current angular velocity of the current position moving towards the target.
     * This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new angle following the smooth damp. The new ang. velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */
    @JvmStatic
    fun Measure<Angle>.smoothDamp(
        target: Measure<Angle>,
        currentVelocity: Cell<Double>,
        smoothTime: Measure<Time>,
        maxVelocity: Number,
        deltaTime: Measure<Time>
    ): Measure<Angle> {
        val res = (this to Degrees).smoothDamp(
            (this + this diff target) to Degrees,
            currentVelocity, smoothTime, maxVelocity, deltaTime
        )
        return Degrees.of(res)
    }

    /**
     * Loops the value, so that it is never larger than length and never smaller than 0.
     *
     * @param length The length of the loop.
     * @return The looped value.
     */
    @JvmStatic
    infix fun Number.repeat(length: Number): Double {
        val tD = this.toDouble()
        val lengthD = length.toDouble()
        return (tD - floor(tD / lengthD) * lengthD) clamp (0.0..lengthD)
    }

    /**
     * PingPongs (bounces) the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param length The length of the pingpong.
     * @return The pingponged value.
     */
    @JvmStatic
    infix fun Number.pingPong(length: Number): Double {
        val lengthD = length.toDouble()
        val repeat = this repeat (lengthD * 2.0)
        return lengthD - abs(repeat - lengthD)
    }

    /**
     * Calculates the shortest difference between two given angles.
     *
     * @param target  The target angle.
     * @return The shortest difference between the two angles.
     */
    @JvmStatic
    infix fun Measure<Angle>.diff(target: Measure<Angle>): Measure<Angle> {
        var delta = ((target - this) to Degrees) repeat 360
        if (delta > 180.0f) delta -= 360.0
        return Degrees.of(delta)
    }

    /**
     * Convert radians to degrees.
     *
     * @return The value in degrees.
     */
    @JvmStatic
    fun Number.radToDeg(): Double {
        return Math.toDegrees(this.toDouble())
    }

    /**
     * Convert degrees to radians.
     *
     * @return The value in radians.
     */
    @JvmStatic
    fun Number.degToRad(): Double {
        return Math.toRadians(this.toDouble())
    }

    /**
     * Orders OpenCV camera points to be in the order top-left, top-right, bottom-right and bottom-left.
     */
    @Suppress("UnusedReceiverParameter")
    @JvmStatic
    fun Array<Any>.orderPoints() = NotImplementedError("Stubbed!")
//    @JvmStatic
//    fun Array<Point>.orderPoints(): Array<Point?> {
//        val orderedPts: Array<Point?> = arrayOfNulls(4)
//
//        // Sum and difference of x and y coordinates
//        val sum = DoubleArray(4)
//        val diff = DoubleArray(4)
//
//        for (i in 0..3) {
//            sum[i] = this[i].x + this[i].y
//            diff[i] = this[i].y - this[i].x
//        }
//
//        // Top-left point has the smallest sum
//        val tlIndex = sum.indexOfMin()
//        orderedPts[0] = this[tlIndex]
//
//        // Bottom-right point has the largest sum
//        val brIndex = sum.indexOfMax()
//        orderedPts[2] = this[brIndex]
//
//        // Top-right point has the smallest difference
//        val trIndex = diff.indexOfMin()
//        orderedPts[1] = this[trIndex]
//
//        // Bottom-left point has the largest difference
//        val blIndex = diff.indexOfMax()
//        orderedPts[3] = this[blIndex]
//
//        return orderedPts
//    }

    /**
     * Determines the index with the lowest value in this array.
     */
    @JvmStatic
    fun DoubleArray.indexOfMin(): Int {
        var index = 0
        var min = this[0]

        for (i in 1 until this.size) {
            if (this[i] < min) {
                min = this[i]
                index = i
            }
        }
        return index
    }

    /**
     * Determines the index with the highest value in this array.
     */
    @JvmStatic
    fun DoubleArray.indexOfMax(): Int {
        var index = 0
        var max = this[0]

        for (i in 1 until this.size) {
            if (this[i] > max) {
                max = this[i]
                index = i
            }
        }
        return index
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
    @JvmStatic
    @Throws(NoInterceptException::class)
    fun lineCircleIntersection(p1: Vector2d, p2: Vector2d, center: Vector2d, radius: Number): Pair<Vector2d, Vector2d> {
        val dx = p2.x - p1.x
        val dy = p2.y - p1.y
        val a = dx * dx + dy * dy
        val b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y))
        val c =
            center.x * center.x + center.y * center.y + p1.x * p1.x + p1.y * p1.y - 2 * (center.x * p1.x + center.y * p1.y) - radius.toDouble() * radius.toDouble()
        val solutions = solveQuadratic(a, b, c)
        if (solutions.isEmpty()) {
            throw NoInterceptException()
        }
        val t1 = solutions[0]
        val t2 = if (solutions.size == 2) solutions[1] else t1
        return Pair(Vector2d(p1.x + t1 * dx, p1.y + t1 * dy), Vector2d(p1.x + t2 * dx, p1.y + t2 * dy))
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
    @JvmStatic
    @Throws(NoInterceptException::class)
    fun lineSegmentCircleIntersection(
        p1: Vector2d,
        p2: Vector2d,
        center: Vector2d,
        radius: Number
    ): Pair<Vector2d, Vector2d> {
        val dx = p2.x - p1.x
        val dy = p2.y - p1.y
        val a = dx * dx + dy * dy
        val b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y))
        val c =
            center.x * center.x + center.y * center.y + p1.x * p1.x + p1.y * p1.y - 2 * (center.x * p1.x + center.y * p1.y) - radius.toDouble() * radius.toDouble()
        val solutions = solveQuadratic(a, b, c)
        if (solutions.isEmpty()) {
            throw NoInterceptException()
        }
        val t1 = solutions[0]
        val t2 = if (solutions.size == 2) solutions[1] else t1
        if ((t1 < 0 || t1 > 1) && (t2 < 0 || t2 > 1)) {
            throw NoInterceptException()
        }
        return Pair(Vector2d(p1.x + t1 * dx, p1.y + t1 * dy), Vector2d(p1.x + t2 * dx, p1.y + t2 * dy))
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
    @JvmStatic
    @Throws(NoInterceptException::class)
    fun lineIntersection(p1: Vector2d, p2: Vector2d, p3: Vector2d, p4: Vector2d): Vector2d {
        val bx = p2.x - p1.x
        val by = p2.y - p1.y
        val dx = p4.x - p3.x
        val dy = p4.y - p3.y

        val bDotDPerp = bx * dy - by * dx
        if (bDotDPerp == 0.0) {
            throw NoInterceptException()
        }
        val cx = p3.x - p1.x
        val cy = p3.y - p1.y
        val t = (cx * dy - cy * dx) / bDotDPerp

        return Vector2d(p1.x + t * bx, p1.y + t * by)
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
    @JvmStatic
    @Throws(NoInterceptException::class)
    fun lineSegmentIntersection(p1: Vector2d, p2: Vector2d, p3: Vector2d, p4: Vector2d): Vector2d {
        val bx = p2.x - p1.x
        val by = p2.y - p1.y
        val dx = p4.x - p3.x
        val dy = p4.y - p3.y

        val bDotDPerp = bx * dy - by * dx
        if (bDotDPerp == 0.0) {
            throw NoInterceptException()
        }
        val cx = p3.x - p1.x
        val cy = p3.y - p1.y
        val t = (cx * dy - cy * dx) / bDotDPerp
        if (t < 0 || t > 1) {
            throw NoInterceptException()
        }
        val u = (cx * by - cy * bx) / bDotDPerp
        if (u < 0 || u > 1) {
            throw NoInterceptException()
        }

        return Vector2d(p1.x + t * bx, p1.y + t * by)
    }

    /**
     * Returns the next power of two that is equal to or larger than the specified value.
     *
     * @return The next power of two.
     */
    @JvmStatic
    fun Int.nextPowerOfTwo(): Int {
        var i = this
        i -= 1
        i = i or (i shr 16)
        i = i or (i shr 8)
        i = i or (i shr 4)
        i = i or (i shr 2)
        i = i or (i shr 1)
        return i + 1
    }

    /**
     * Returns the closest power of two that is equal to or larger than the specified value.
     *
     * @return The closest power of two.
     */
    @JvmStatic
    fun Int.closestPowerOfTwo(): Int {
        val nextPower = this.nextPowerOfTwo()
        val prevPower = nextPower shr 1
        return if (this - prevPower < nextPower - this) prevPower
        else nextPower
    }

    /**
     * Returns whether the given value is a power of two.
     *
     * @return Whether the value is a power of two.
     */
    @JvmStatic
    fun Int.isPowerOfTwo(): Boolean {
        return (this and (this - 1)) == 0
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    @JvmStatic
    fun inverseLerp(startValue: Number, endValue: Number, q: Number): Double {
        val startValueD = startValue.toDouble()
        val totalRange = endValue.toDouble() - startValueD
        if (totalRange <= 0) {
            return 0.0
        }
        val queryToStart = q.toDouble() - startValueD
        if (queryToStart <= 0) {
            return 0.0
        }
        return queryToStart / totalRange
    }

    /**
     * Return where within interpolation range [0, 1] q is between the start and end pair.
     *
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    @JvmStatic
    fun ClosedFloatingPointRange<Double>.inverseLerp(q: Number): Double {
        val startValueD = this.start
        val totalRange = this.endInclusive - startValueD
        if (totalRange <= 0) {
            return 0.0
        }
        val queryToStart = q.toDouble() - startValueD
        if (queryToStart <= 0) {
            return 0.0
        }
        return queryToStart / totalRange
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance.
     *
     * @param expected  The expected value
     * @param tolerance The allowed difference between the actual and the expected value
     * @return Whether the actual value is within the allowed tolerance
     */
    @JvmStatic
    fun Number.isNear(expected: Number, tolerance: Number): Boolean {
        val toleranceD = tolerance.toDouble()
        require(!(toleranceD < 0)) { "Tolerance must be a non-negative number!" }
        return abs(expected.toDouble() - this.toDouble()) < toleranceD
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance. Supports
     * continuous input for cases like absolute encoders.
     *
     * Continuous input means that the min and max value are considered to be the same point, and
     * tolerances can be checked across them. A common example would be for absolute encoders: calling
     * `isNear(2, 359, 5, 0, 360)` returns `true` because 359 is 1 away from 360 (which is treated as the
     * same as 0) and 2 is 2 away from 0, adding up to an error of 3 degrees, which is within the
     * given tolerance of 5.
     *
     * @param expected  The expected value
     * @param tolerance The allowed difference between the actual and the expected value
     * @param min       Smallest value before wrapping around to the largest value
     * @param max       Largest value before wrapping around to the smallest value
     * @return Whether the actual value is within the allowed tolerance
     */
    @JvmStatic
    fun Number.isNear(expected: Number, tolerance: Number, min: Number, max: Number): Boolean {
        val toleranceD = tolerance.toDouble()
        require(!(toleranceD < 0)) { "Tolerance must be a non-negative number!" }
        // Max error is exactly halfway between the min and max
        val errorBound = (max.toDouble() - min.toDouble()) / 2.0
        val error = (expected.toDouble() - this.toDouble()) wrap (-errorBound..errorBound)
        return abs(error) < toleranceD
    }

    /**
     * Exception thrown if no intercept is found when using the intersection methods of this class.
     */
    class NoInterceptException : RuntimeException("Intercept calculation failed due to no intercepts or an edge case.")

    /**
     * 2π
     */
    const val TWO_PI = 2.0 * PI
}
