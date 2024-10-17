package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf

/**
 * Linear heading interpolator for time-optimal transitions between poses.
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param startHeading start heading
 * @param angle angle to sweep through (can be greater than a revolution)
 * @since 6.0.0
 */
class LinearInterpolator(private val startHeading: Double, private val angle: Double) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double) =
        Mathf.normaliseRadians(startHeading + s / curve.length() * angle)

    override fun internalDeriv(s: Double, t: Double) = angle / curve.length()

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}