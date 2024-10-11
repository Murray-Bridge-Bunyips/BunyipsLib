package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.murraybridgebunyips.bunyipslib.external.Mathf
import org.murraybridgebunyips.bunyipslib.external.units.UnaryFunction
import java.util.function.Supplier

/**
 * Default localizer for Tank drives based on the drive encoders and (optionally) a heading sensor.
 *
 * @param trackWidthInches lateral distance *in inches* between pairs of wheels on different sides of the robot
 * @param ticksToInches conversion function for all the encoders to convert ticks to inches, see `EncoderTicks.toInches`
 * @param wheelPositions 2-wide list supplier as calculated by the average position of all left and all right wheels
 * @param wheelVelocities optional wheel position deltas or wheel velocities in the sequence of [wheelPositions]
 * @param headingSensor external heading supplier + external heading velocity supplier, *in radians*
 * @since 5.1.0
 */
class TankLocalizer @JvmOverloads constructor(
    private val trackWidthInches: Double,
    private val ticksToInches: UnaryFunction,
    private val wheelPositions: Supplier<List<Number>>,
    private val headingSensor: Pair<Supplier<Double>, Supplier<Double>?>? = null,
    private val wheelVelocities: Supplier<List<Number>>? = null,
) : Localizer {
    private var offset: Double = 0.0
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            offset = value.heading
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    override fun update() {
        val wheelPositions = wheelPositions.get().map { ticksToInches.apply(it.toDouble()) }
        val extHeading = headingSensor?.first?.get()
            ?.plus(offset)?.let { v -> Mathf.inputModulus(v, 0.0, 2.0 * Math.PI) } ?: Double.NaN
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelDeltas, trackWidthInches)
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

        val wheelVelocities = wheelVelocities?.get()?.map { ticksToInches.apply(it.toDouble()) }
        val extHeadingVel = headingSensor?.second?.get()
        if (wheelVelocities != null) {
            poseVelocity = TankKinematics.wheelToRobotVelocities(wheelVelocities, trackWidthInches)
            if (headingSensor != null && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }
}