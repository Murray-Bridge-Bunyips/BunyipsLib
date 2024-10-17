package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf

/**
 * Constant heading interpolator used for arbitrary holonomic translations.
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param heading heading to maintain
 * @since 6.0.0
 */
class ConstantInterpolator(val heading: Double) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double): Double = Mathf.normaliseRadians(heading)

    override fun internalDeriv(s: Double, t: Double) = 0.0

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}