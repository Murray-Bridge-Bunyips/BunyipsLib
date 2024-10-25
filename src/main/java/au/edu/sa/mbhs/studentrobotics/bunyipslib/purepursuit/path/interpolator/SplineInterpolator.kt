package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapDeltaRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.ParametricCurve
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.QuinticPolynomial

/**
 * Spline heading interpolator for transitioning smoothly between headings without violating continuity (and hence
 * allowing for integration into longer profiles).
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param startHeading start heading
 * @param endHeading end heading
 * @param startHeadingDeriv start heading deriv (advanced)
 * @param startHeadingSecondDeriv start heading second deriv (advanced)
 * @param endHeadingDeriv start heading deriv (advanced)
 * @param endHeadingSecondDeriv start heading second deriv (advanced)
 * @since 6.0.0
 */
// note: the spline parameter is transformed linearly into a pseudo-arclength parameter
class SplineInterpolator @JvmOverloads constructor(
    private val startHeading: Double,
    private val endHeading: Double,
    private val startHeadingDeriv: Double? = null,
    private val startHeadingSecondDeriv: Double? = null,
    private val endHeadingDeriv: Double? = null,
    private val endHeadingSecondDeriv: Double? = null
) : HeadingInterpolator() {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial

    override fun init(curve: ParametricCurve) {
        super.init(curve)

        tangentInterpolator.init(this.curve)

        val len = curve.length()

        val headingDelta = (endHeading - startHeading).wrapDeltaRadians()

        headingSpline = QuinticPolynomial(
            0.0,
            (startHeadingDeriv ?: curve.tangentAngleDeriv(0.0, 0.0)) * len,
            (startHeadingSecondDeriv ?: curve.tangentAngleSecondDeriv(0.0, 0.0)) * len * len,
            headingDelta,
            (endHeadingDeriv ?: curve.tangentAngleDeriv(len, 1.0)) * len,
            (endHeadingSecondDeriv ?: curve.tangentAngleSecondDeriv(len, 1.0)) * len * len
        )
    }

    override fun internalGet(s: Double, t: Double) =
        (startHeading + headingSpline[s / curve.length()]).wrapRadians()

    override fun internalDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.deriv(s / len) / len
    }

    override fun internalSecondDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.secondDeriv(s / len) / (len * len)
    }
}