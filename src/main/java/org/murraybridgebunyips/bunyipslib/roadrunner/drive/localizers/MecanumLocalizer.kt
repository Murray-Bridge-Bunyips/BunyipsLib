package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.murraybridgebunyips.bunyipslib.Reference
import java.util.function.Supplier

/**
 * Default localizer for Mecanum drives based on the drive encoders and (optionally) a heading sensor.
 *
 * @param trackWidth lateral distance *in inches* between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot *in inches*
 * @param lateralMultiplier multiplicative gain to adjust for systematic, proportional lateral error (gain greater
 *      than 1.0 corresponds to overcompensation).
 * @param wheelPositions current position of all four wheels, in order `frontLeft`, `backLeft`, `backRight`, `frontRight`
 * @param wheelVelocities optional wheel position deltas or wheel velocities of all four wheels in the order of [wheelPositions]
 * @param headingSensor external heading supplier/consumer + external heading velocity supplier, *in radians*
 * @since 5.1.0
 */
class MecanumLocalizer @JvmOverloads constructor(
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth,
    private val lateralMultiplier: Double = 1.0,
    private val wheelPositions: Supplier<List<Double>>,
    private val wheelVelocities: Supplier<List<Double>>? = null,
    private val headingSensor: Pair<Reference<Double>, Supplier<Double>>? = null
) : Localizer {
    init {
        headingSensor?.first?.setIfNotPresent(0.0)
    }

    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            headingSensor?.first?.set(value.heading)
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    override fun update() {
        val wheelPositions = wheelPositions.get()
        val extHeading = headingSensor?.first?.require() ?: Double.NaN
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas,
                trackWidth,
                wheelBase,
                lateralMultiplier
            )
            val finalHeadingDelta = if (headingSensor != null) {
                Angle.normDelta(extHeading - lastExtHeading)
            } else {
                robotPoseDelta.heading
            }
            _poseEstimate = Kinematics.relativeOdometryUpdate(
                _poseEstimate,
                Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
            )
        }

        val wheelVelocities = wheelVelocities?.get()
        val extHeadingVel = headingSensor?.second?.get()
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                wheelVelocities,
                trackWidth,
                wheelBase,
                lateralMultiplier
            )
            if (headingSensor != null && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }
}