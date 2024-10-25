package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapRadians

/**
 * Tangent (system) interpolator for tank/differential and other nonholonomic drives.
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param offset tangent heading offset
 * @since 6.0.0
 */
class TangentInterpolator @JvmOverloads constructor(
    internal val offset: Double = 0.0
) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double) = (offset + curve.tangentAngle(s, t)).wrapRadians()

    override fun internalDeriv(s: Double, t: Double) = curve.tangentAngleDeriv(s, t)

    override fun internalSecondDeriv(s: Double, t: Double) = curve.tangentAngleSecondDeriv(s, t)
}