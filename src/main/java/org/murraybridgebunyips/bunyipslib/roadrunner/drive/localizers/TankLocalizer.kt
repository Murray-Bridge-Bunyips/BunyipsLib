package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.murraybridgebunyips.bunyipslib.Reference
import java.util.function.Supplier

/**
 * Default localizer for Tank drives based on the drive encoders and (optionally) a heading sensor.
 *
 * @param trackWidth lateral distance *in inches* between pairs of wheels on different sides of the robot
 * @param wheelPositions 2-wide list supplier as calculated by the average position of all left and all right wheels
 * @param wheelVelocities optional wheel position deltas or wheel velocities in the sequence of [wheelPositions]
 * @param headingSensor external heading supplier/consumer + external heading velocity supplier, *in radians*
 * @since 5.1.0
 */
class TankLocalizer @JvmOverloads constructor(
    private val trackWidth: Double,
    private val wheelPositions: Supplier<List<Number>>,
    private val wheelVelocities: Supplier<List<Number>>? = null,
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
                .map { it.first.toDouble() - it.second }
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelDeltas, trackWidth)
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
            poseVelocity = TankKinematics.wheelToRobotVelocities(wheelVelocities.map { it.toDouble() }, trackWidth)
            if (headingSensor != null && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions.map { it.toDouble() }
        lastExtHeading = extHeading
    }
}