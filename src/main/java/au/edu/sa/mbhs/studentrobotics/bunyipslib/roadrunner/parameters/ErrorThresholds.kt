package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.DegreesPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity

/**
 * Data class for storing constants related to extra correction for Mecanum drive stabilization.
 * This extra correction is not possible in Tank drives due to the non-holonomic constraints.
 *
 * @param stabilizationTimeout the maximum buffer of extra time that can be used on top of the task to stabilize the robot
 * @param maxTranslationalError the maximum translational error
 * @param minVelStab the minimum velocity to be considered for stabilization
 * @param maxAngularError the maximum angular error
 * @param minAngVelStab the minimum angular velocity to be considered for stabilization
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
data class ErrorThresholds(
    val stabilizationTimeout: Measure<Time>,
    val maxTranslationalError: Measure<Distance>,
    val minVelStab: Measure<Velocity<Distance>>,
    val maxAngularError: Measure<Angle>,
    val minAngVelStab: Measure<Velocity<Angle>>
) {
    companion object {
        /**
         * Default error thresholds for Mecanum drive stabilization.
         */
        @JvmField
        val DEFAULT = ErrorThresholds(
            Seconds.of(0.5),
            Inches.of(2.0), InchesPerSecond.of(0.5),
            Degrees.of(1.0), DegreesPerSecond.of(5.0)
        )

        /**
         * No error thresholds which disable the stabilization elements.
         */
        @JvmField
        val NONE = ErrorThresholds(
            Seconds.of(0.0),
            Inches.of(0.0), InchesPerSecond.of(0.0),
            Degrees.of(0.0), DegreesPerSecond.of(0.0)
        )
    }
}