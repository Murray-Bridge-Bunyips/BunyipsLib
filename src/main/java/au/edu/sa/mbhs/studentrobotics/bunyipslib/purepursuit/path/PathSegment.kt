package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path

import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.HeadingInterpolator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.TangentInterpolator
import com.acmerobotics.roadrunner.Pose2d

/**
 * Path segment composed of a parametric curve and heading interpolator.
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param curve parametric curve
 * @param interpolator heading interpolator
 * @since 6.0.0
 */
class PathSegment @JvmOverloads constructor(
    val curve: ParametricCurve,
    val interpolator: HeadingInterpolator = TangentInterpolator()
) {
    init {
        interpolator.init(curve)
    }

    /**
     * The length of the path segment.
     */
    fun length() = curve.length()

    /**
     * Get a position along the segment.
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)) = Pose2d(curve[s, t], interpolator[s, t])

    /**
     * Get a derivative along the segment.
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)) = Pose2d(curve.deriv(s, t), interpolator.deriv(s, t))

    /**
     * Get a second derivative along the segment.
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)) =
        Pose2d(curve.secondDeriv(s, t), interpolator.secondDeriv(s, t))

    /**
     * Get a tangent angle along the segment.
     */
    @JvmOverloads
    fun tangentAngle(s: Double, t: Double = reparam(s)) = curve.tangentAngle(s, t)

    @JvmOverloads
    internal fun internalDeriv(s: Double, t: Double = reparam(s)) =
        Pose2d(curve.internalDeriv(t), interpolator.internalDeriv(s, t))

    @JvmOverloads
    internal fun internalSecondDeriv(s: Double, t: Double = reparam(s)) =
        Pose2d(curve.internalSecondDeriv(t), interpolator.internalDeriv(s, t))

    /**
     * Re-parameterise to t
     */
    fun reparam(s: Double) = curve.reparam(s)

    /**
     * Returns the start pose.
     */
    fun start() = get(0.0)

    /**
     * Returns the start pose derivative.
     */
    fun startDeriv() = deriv(0.0)

    /**
     * Returns the start pose second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0)

    /**
     * Returns the start tangent angle.
     */
    fun startTangentAngle() = tangentAngle(0.0)

    internal fun startInternalDeriv() = internalDeriv(0.0)

    internal fun startInternalSecondDeriv() = internalSecondDeriv(0.0)

    /**
     * Returns the end pose.
     */
    fun end() = get(length())

    /**
     * Returns the end pose derivative.
     */
    fun endDeriv() = deriv(length())

    /**
     * Returns the end pose second derivative.
     */
    fun endSecondDeriv() = secondDeriv(length())

    /**
     * Returns the end tangent angle.
     */
    fun endTangentAngle() = tangentAngle(length())

    internal fun endInternalDeriv() = internalDeriv(length())

    internal fun endInternalSecondDeriv() = internalSecondDeriv(length())
}