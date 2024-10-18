package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints.Turn
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint

/**
 * Extension of a RoadRunner trajectory builder to provide WPIUnits and task building support.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
class TaskBuilder(constants: Constants, startPose: Pose2d, poseMap: PoseMap) {
    private var turnConstraints: TurnConstraints = constants.baseTurnConstraints
    private var velConstraints: VelConstraint = constants.baseVelConstraint
    private var accelConstraint: AccelConstraint = constants.baseAccelConstraint
    private val builder: TrajectoryActionBuilder = TrajectoryActionBuilder(
        constants.turnActionFactory,
        constants.trajectoryActionFactory,
        constants.trajectoryBuilderParams,
        startPose,
        constants.beginEndVel,
        constants.baseTurnConstraints,
        constants.baseVelConstraint,
        constants.baseAccelConstraint,
        poseMap
    )

    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() = apply { builder.endTrajectory() }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds action [a] next.
     */
    fun stopAndAdd(a: Action) = apply { builder.stopAndAdd(a) }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds the instant function [f] next.
     */
    fun stopAndAdd(f: InstantFunction) = apply { stopAndAdd(InstantAction(f)) }

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double) = apply { builder.waitSeconds(t) }

    /**
     * Waits [t] time units of [unit].
     */
    @JvmOverloads
    fun waitFor(t: Double, unit: Time = Seconds) = apply { waitSeconds(unit.of(t).inUnit(Seconds)) }

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    @JvmOverloads
    fun afterDisp(ds: Double, unit: Distance = Inches, a: Action) = apply { builder.afterDisp(unit.of(ds).inUnit(Inches), a) }

    /**
     * Schedules function [f] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    @JvmOverloads
    fun afterDisp(ds: Double, unit: Distance = Inches, f: InstantFunction) = apply { afterDisp(ds, unit, InstantAction(f)) }

    /**
     * Schedules action [a] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    @JvmOverloads
    fun afterTime(dt: Double, unit: Time = Seconds, a: Action) = apply { builder.afterTime(unit.of(dt).inUnit(Seconds), a) }

    /**
     * Schedules function [f] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    @JvmOverloads
    fun afterTime(dt: Double, unit: Time = Seconds, f: InstantFunction) = apply { afterTime(dt, unit, InstantAction(f)) }

    /**
     * Sets the tangent of the builder.
     */
    fun setTangent(r: Rotation2d) = apply { builder.setTangent(r) }

    /**
     * Sets the tangent of the builder.
     */
    @JvmOverloads
    fun setTangent(r: Double, unit: Angle = Radians) = apply { builder.setTangent(unit.of(r).inUnit(Radians)) }

    /**
     * Set the reversed tangent state of the builder.
     */
    fun setReversed(reversed: Boolean) = apply { builder.setReversed(reversed) }

    /**
     * Turn the robot by [angle] in [unit].
     */
    @JvmOverloads
    fun turn(angle: Double, unit: Angle = Radians) =
        apply { builder.turn(unit.of(angle).inUnit(Radians), turnConstraints) }

    /**
     * Turn the robot to the [heading], specified as a [Rotation2d] in radians.
     */
    fun turnTo(heading: Rotation2d) = apply { builder.turnTo(heading, turnConstraints) }

    /**
     * Turn the robot to the [heading] in [unit].
     */
    @JvmOverloads
    fun turnTo(heading: Double, unit: Angle = Radians) =
        apply { builder.turnTo(unit.of(heading).inUnit(Radians), turnConstraints) }

    // TODO: vel and accel related methods


    // TODO: united constraints class
    /**
     * Set the turn constraints for future builder instructions in units of inches.
     */
    fun setTurnConstraints(constraintsInches: TurnConstraints) =
        apply { turnConstraints = constraintsInches }

    /**
     * Set the turn constraints as defined by a [Turn] object.
     */
    fun setTurnConstraints(constraints: Turn) =
        apply {
           turnConstraints = TurnConstraints(
               constraints.maxAngVelRadsPerSec ?: turnConstraints.maxAngVel,
               constraints.minAngAccelRadsPerSecSquared ?: turnConstraints.minAngAccel,
               constraints.maxAngAccelRadsPerSecSquared ?: turnConstraints.maxAngAccel
           )
        }

    // TODO: RoadRunner utils for building
}
