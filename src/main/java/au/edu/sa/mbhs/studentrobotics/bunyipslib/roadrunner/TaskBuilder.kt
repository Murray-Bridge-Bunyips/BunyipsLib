package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner

import au.edu.sa.mbhs.studentrobotics.bunyipslib.AutonomousBunyipsOpMode
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Reference
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints.Accel
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints.Turn
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints.Vel
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ActionTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint

/**
 * Extension of a RoadRunner trajectory builder to provide WPIUnits and task building support.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
class TaskBuilder @JvmOverloads constructor(
    private val constants: Constants,
    startPose: Pose2d,
    poseMap: PoseMap,
    private val caller: RoadRunnerDrive? = null
) {
    private var turnConstraints = constants.baseTurnConstraints
    private var velConstraints = constants.baseVelConstraint
    private var accelConstraints = constants.baseAccelConstraint
    private var timeout: Measure<Time>? = null
    private var name: String? = null
    private var priority = AutonomousBunyipsOpMode.TaskPriority.NORMAL
    private var builder = TrajectoryActionBuilder(
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
    fun endTrajectory() = apply { builder = builder.endTrajectory() }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds action [a] next.
     */
    fun stopAndAdd(a: Action) = apply { builder = builder.stopAndAdd(a) }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds the instant function [f] next.
     */
    fun stopAndAdd(f: InstantFunction) = apply { stopAndAdd(InstantAction(f)) }

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double) = apply { builder = builder.waitSeconds(t) }

    /**
     * Waits [t] time units of [unit].
     */
    @JvmOverloads
    fun waitFor(t: Double, unit: Time = Seconds) = apply { waitSeconds(unit.of(t) to Seconds) }

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    @JvmOverloads
    fun afterDisp(ds: Double, unit: Distance = Inches, a: Action) =
        apply { builder = builder.afterDisp(unit.of(ds) to Inches, a) }

    /**
     * Schedules function [f] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    @JvmOverloads
    fun afterDisp(ds: Double, unit: Distance = Inches, f: InstantFunction) =
        apply { afterDisp(ds, unit, InstantAction(f)) }

    /**
     * Schedules action [a] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    @JvmOverloads
    fun afterTime(dt: Double, unit: Time = Seconds, a: Action) =
        apply { builder = builder.afterTime(unit.of(dt) to Seconds, a) }

    /**
     * Schedules function [f] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    @JvmOverloads
    fun afterTime(dt: Double, unit: Time = Seconds, f: InstantFunction) =
        apply { afterTime(dt, unit, InstantAction(f)) }

    /**
     * Sets the tangent of the builder.
     */
    fun setTangent(r: Rotation2d) = apply { builder = builder.setTangent(r) }

    /**
     * Sets the tangent of the builder.
     */
    @JvmOverloads
    fun setTangent(r: Double, unit: Angle = Radians) =
        apply { builder = builder.setTangent(unit.of(r) to Radians) }

    /**
     * Set the reversed tangent state of the builder.
     */
    fun setReversed(reversed: Boolean) = apply { builder = builder.setReversed(reversed) }

    /**
     * Turn the robot by [angle] in [unit].
     */
    @JvmOverloads
    fun turn(angle: Double, unit: Angle = Radians) =
        apply { builder = builder.turn(unit.of(angle) to Radians, turnConstraints) }

    /**
     * Turn the robot to the [heading], specified as a [Rotation2d] in radians.
     */
    fun turnTo(heading: Rotation2d) = apply { builder = builder.turnTo(heading, turnConstraints) }

    /**
     * Turn the robot to the [heading] in [unit].
     */
    @JvmOverloads
    fun turnTo(heading: Double, unit: Angle = Radians) =
        apply { builder = builder.turnTo(unit.of(heading) to Radians, turnConstraints) }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading.
     */
    @JvmOverloads
    fun lineToX(posX: Double, unit: Distance = Inches) =
        apply { builder = builder.lineToX(unit.of(posX) to Inches, velConstraints, accelConstraints) }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading.
     */
    @JvmOverloads
    fun lineToXConstantHeading(posX: Double, unit: Distance = Inches) =
        apply {
            builder = builder.lineToXConstantHeading(unit.of(posX) to Inches, velConstraints, accelConstraints)
        }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading while linearly interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToXLinearHeading(posX: Double, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.lineToXLinearHeading(
                posUnit.of(posX) to Inches,
                heading,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading while linearly interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToXLinearHeading(posX: Double, posUnit: Distance = Inches, heading: Double, headingUnit: Angle = Radians) =
        apply {
            builder = builder.lineToXLinearHeading(
                posUnit.of(posX) to Inches,
                headingUnit.of(heading) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading while spline interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToXSplineHeading(posX: Double, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.lineToXSplineHeading(
                posUnit.of(posX) to Inches,
                heading,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posX] coordinate in the direction of the current heading while spline interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToXSplineHeading(posX: Double, posUnit: Distance = Inches, heading: Double, headingUnit: Angle = Radians) =
        apply {
            builder = builder.lineToXSplineHeading(
                posUnit.of(posX) to Inches,
                headingUnit.of(heading) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading.
     */
    @JvmOverloads
    fun lineToY(posY: Double, unit: Distance = Inches) =
        apply { builder = builder.lineToY(unit.of(posY) to Inches, velConstraints, accelConstraints) }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading.
     */
    @JvmOverloads
    fun lineToYConstantHeading(posY: Double, unit: Distance = Inches) =
        apply {
            builder = builder.lineToYConstantHeading(unit.of(posY) to Inches, velConstraints, accelConstraints)
        }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading while linearly interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToYLinearHeading(posY: Double, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.lineToYLinearHeading(
                posUnit.of(posY) to Inches,
                heading,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading while linearly interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToYLinearHeading(posY: Double, posUnit: Distance = Inches, heading: Double, headingUnit: Angle = Radians) =
        apply {
            builder = builder.lineToYLinearHeading(
                posUnit.of(posY) to Inches,
                headingUnit.of(heading) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading while spline interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToYSplineHeading(posY: Double, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.lineToYSplineHeading(
                posUnit.of(posY) to Inches,
                heading,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [posY] coordinate in the direction of the current heading while spline interpolating the
     * heading to [heading].
     */
    @JvmOverloads
    fun lineToYSplineHeading(posY: Double, posUnit: Distance = Inches, heading: Double, headingUnit: Angle = Radians) =
        apply {
            builder = builder.lineToYSplineHeading(
                posUnit.of(posY) to Inches,
                headingUnit.of(heading) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while maintaining the current heading.
     */
    @JvmOverloads
    fun strafeTo(pos: Vector2d, unit: Distance = Inches) =
        apply {
            builder = builder.strafeTo(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while maintaining the current heading.
     */
    @JvmOverloads
    fun strafeToConstantHeading(pos: Vector2d, unit: Distance = Inches) =
        apply {
            builder = builder.strafeToConstantHeading(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while linearly interpolating the heading to [heading].
     */
    @JvmOverloads
    fun strafeToLinearHeading(pos: Vector2d, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.strafeToLinearHeading(
                Vector2d(
                    posUnit.of(pos.x) to Inches,
                    posUnit.of(pos.y) to Inches
                ), heading, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while linearly interpolating the heading to [heading].
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        posUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians
    ) =
        apply {
            builder = builder.strafeToLinearHeading(
                Vector2d(
                    posUnit.of(pos.x) to Inches,
                    posUnit.of(pos.y) to Inches
                ), headingUnit.of(heading) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while spline interpolating the heading to [heading].
     */
    @JvmOverloads
    fun strafeToSplineHeading(pos: Vector2d, posUnit: Distance = Inches, heading: Rotation2d) =
        apply {
            builder = builder.strafeToSplineHeading(
                Vector2d(
                    posUnit.of(pos.x) to Inches,
                    posUnit.of(pos.y) to Inches
                ), heading, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [pos] vector while spline interpolating the heading to [heading].
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        posUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians
    ) =
        apply {
            builder = builder.strafeToSplineHeading(
                Vector2d(
                    posUnit.of(pos.x) to Inches,
                    posUnit.of(pos.y) to Inches
                ), headingUnit.of(heading) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [pos] in a spline path while following the specified [tangent].
     */
    @JvmOverloads
    fun splineTo(pos: Vector2d, unit: Distance = Inches, tangent: Rotation2d) =
        apply {
            builder = builder.splineTo(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                tangent,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] in a spline path while following the specified [tangent].
     */
    @JvmOverloads
    fun splineTo(pos: Vector2d, unit: Distance = Inches, tangent: Double, tangentUnit: Angle = Radians) =
        apply {
            builder = builder.splineTo(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                tangentUnit.of(tangent) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] in a spline path while maintaining the current heading.
     */
    @JvmOverloads
    fun splineToConstantHeading(pos: Vector2d, unit: Distance = Inches, tangent: Rotation2d) =
        apply {
            builder = builder.splineToConstantHeading(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                tangent,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [pos] in a spline path while maintaining the current heading.
     */
    @JvmOverloads
    fun splineToConstantHeading(pos: Vector2d, unit: Distance = Inches, tangent: Double, tangentUnit: Angle = Radians) =
        apply {
            builder = builder.splineToConstantHeading(
                Vector2d(unit.of(pos.x) to Inches, unit.of(pos.y) to Inches),
                tangentUnit.of(tangent) to Radians,
                velConstraints,
                accelConstraints
            )
        }

    /**
     * Move to the specified [vector] tangent to [heading] in a spline path while linearly interpolating the robot heading to [tangent].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        vector: Vector2d,
        vectorUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians,
        tangent: Rotation2d
    ) =
        apply {
            builder = builder.splineToLinearHeading(
                Pose2d(
                    vectorUnit.of(vector.x) to Inches,
                    vectorUnit.of(vector.y) to Inches,
                    headingUnit.of(heading) to Radians
                ), tangent, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [vector] tangent to [heading] in a spline path while linearly interpolating the robot heading to [tangent].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        vector: Vector2d,
        vectorUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians,
        tangent: Double,
        tangentUnit: Angle = Radians
    ) =
        apply {
            builder = builder.splineToLinearHeading(
                Pose2d(
                    vectorUnit.of(vector.x) to Inches,
                    vectorUnit.of(vector.y) to Inches,
                    headingUnit.of(heading) to Radians
                ), tangentUnit.of(tangent) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [vector] tangent to [heading] in a spline path while spline interpolating the robot heading to [tangent].
     */
    @JvmOverloads
    fun splineToSplineHeading(
        vector: Vector2d,
        vectorUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians,
        tangent: Rotation2d
    ) =
        apply {
            builder = builder.splineToSplineHeading(
                Pose2d(
                    vectorUnit.of(vector.x) to Inches,
                    vectorUnit.of(vector.y) to Inches,
                    headingUnit.of(heading) to Radians
                ), tangent, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [vector] tangent to [heading] in a spline path while spline interpolating the robot heading to [tangent].
     */
    @JvmOverloads
    fun splineToSplineHeading(
        vector: Vector2d,
        vectorUnit: Distance = Inches,
        heading: Double,
        headingUnit: Angle = Radians,
        tangent: Double,
        tangentUnit: Angle = Radians
    ) =
        apply {
            builder = builder.splineToSplineHeading(
                Pose2d(
                    vectorUnit.of(vector.x) to Inches,
                    vectorUnit.of(vector.y) to Inches,
                    headingUnit.of(heading) to Radians
                ), tangentUnit.of(tangent) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [poseHeadingRad] in a spline path while linearly interpolating the heading to [tangent].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        poseHeadingRad: Pose2d,
        vectorUnit: Distance = Inches,
        tangent: Rotation2d
    ) =
        apply {
            builder = builder.splineToLinearHeading(
                Pose2d(
                    Vector2d(
                        vectorUnit.of(poseHeadingRad.position.x) to Inches,
                        vectorUnit.of(poseHeadingRad.position.y) to Inches
                    ),
                    poseHeadingRad.heading
                ), tangent, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [poseHeadingRad] in a spline path while linearly interpolating the heading to [tangent].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        poseHeadingRad: Pose2d,
        vectorUnit: Distance = Inches,
        tangent: Double,
        tangentUnit: Angle = Radians
    ) =
        apply {
            builder = builder.splineToLinearHeading(
                Pose2d(
                    Vector2d(
                        vectorUnit.of(poseHeadingRad.position.x) to Inches,
                        vectorUnit.of(poseHeadingRad.position.y) to Inches
                    ),
                    poseHeadingRad.heading
                ), tangentUnit.of(tangent) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [poseHeadingRad] in a spline path while spline interpolating the heading to [tangent].
     */
    @JvmOverloads
    fun splineToSplineHeading(
        poseHeadingRad: Pose2d,
        vectorUnit: Distance = Inches,
        tangent: Rotation2d
    ) =
        apply {
            builder = builder.splineToSplineHeading(
                Pose2d(
                    Vector2d(
                        vectorUnit.of(poseHeadingRad.position.x) to Inches,
                        vectorUnit.of(poseHeadingRad.position.y) to Inches
                    ),
                    poseHeadingRad.heading
                ),
                tangent, velConstraints, accelConstraints
            )
        }

    /**
     * Move to the specified [poseHeadingRad] in a spline path while spline interpolating the heading to [tangent].
     */
    @JvmOverloads
    fun splineToSplineHeading(
        poseHeadingRad: Pose2d,
        vectorUnit: Distance = Inches,
        tangent: Double,
        tangentUnit: Angle = Radians
    ) =
        apply {
            builder = builder.splineToSplineHeading(
                Pose2d(
                    Vector2d(
                        vectorUnit.of(poseHeadingRad.position.x) to Inches,
                        vectorUnit.of(poseHeadingRad.position.y) to Inches
                    ),
                    poseHeadingRad.heading
                ), tangentUnit.of(tangent) to Radians, velConstraints, accelConstraints
            )
        }

    /**
     * Creates a new builder with the same settings and default constraints at the current pose, tangent.
     */
    fun fresh(): TaskBuilder {
        endTrajectory()
        val bClass = builder::class.java
        val lastPoseUnmappedField = bClass.getDeclaredField("lastPoseUnmapped")
        val lastTangentField = bClass.getDeclaredField("lastTangent")
        lastPoseUnmappedField.isAccessible = true
        lastTangentField.isAccessible = true
        val lastPoseUnmapped = lastPoseUnmappedField.get(builder) as Pose2d
        val lastTangent = lastTangentField.get(builder) as Rotation2d

        return TaskBuilder(constants, lastPoseUnmapped, builder.poseMap, caller).setTangent(lastTangent)
    }

    /**
     * Set the turn constraints for future builder instructions in units of inches.
     */
    fun setTurnConstraints(constraintsInches: TurnConstraints) =
        apply { turnConstraints = constraintsInches }

    /**
     * Set the turn constraints as defined by a [Turn] object.
     */
    fun setTurnConstraints(constraints: Turn) = apply { turnConstraints = constraints.getOrDefault(turnConstraints) }

    /**
     * Reset the turn constraints to default.
     */
    fun resetTurnConstraints() = apply { turnConstraints = constants.baseTurnConstraints }

    /**
     * Set the velocity constraints for future builder instructions in units of inches.
     */
    fun setVelConstraints(velConstraintsInches: VelConstraint) = apply { velConstraints = velConstraintsInches }

    /**
     * Set the velocity constraints as defined by a [Vel] object.
     */
    fun setVelConstraints(constraints: Vel) =
        apply {
            val (translation, angular) = if (velConstraints is MinVelConstraint) {
                val minVelConstraint = velConstraints as MinVelConstraint
                minVelConstraint.constraints[0] to minVelConstraint.constraints[1]
            } else {
                TranslationalVelConstraint(Double.MAX_VALUE) to AngularVelConstraint(Double.MAX_VALUE)
            }
            velConstraints = constraints.getOrDefault(translation, angular)
        }

    /**
     * Reset the velocity constraints to default.
     */
    fun resetVelConstraints() = apply { velConstraints = constants.baseVelConstraint }

    /**
     * Set the acceleration constraints for future builder instructions in units of inches.
     */
    fun setAccelConstraints(accelConstraints: AccelConstraint) = apply { this.accelConstraints = accelConstraints }

    /**
     * Set the acceleration constraints as defined by an [Accel] object.
     */
    fun setAccelConstraints(constraints: Accel) =
        apply {
            accelConstraints = constraints.getOrDefault(
                (accelConstraints as? ProfileAccelConstraint) ?: ProfileAccelConstraint(
                    -Double.MAX_VALUE,
                    Double.MAX_VALUE
                )
            )
        }

    /**
     * Reset the acceleration constraints to default.
     */
    fun resetAccelConstraints() = apply { accelConstraints = constants.baseAccelConstraint }

    /**
     * Set the timeout for the underlying task.
     */
    fun withTimeout(timeout: Measure<Time>) = apply { this.timeout = timeout }

    /**
     * Set the name for the underlying task.
     */
    fun withName(name: String) = apply { this.name = name }

    /**
     * Set the priority for the underlying task if using [addTask].
     */
    fun withPriority(priority: AutonomousBunyipsOpMode.TaskPriority) = apply { this.priority = priority }

    /**
     * Build the current trajectory and return it as a [Task]/[Action].
     */
    fun build() = ActionTask(builder.build()).also {
        if (name != null)
            it.withName(name)
        if (timeout != null)
            it.withTimeout(timeout!!)
        if (caller != null && caller is BunyipsSubsystem)
            it.onSubsystem(caller)
    }

    /**
     * Build the current trajectory and return it as a [Task]/[Action].
     * This overload also stores the last *unmapped* spliced pose in the given [Reference] object for manual chaining.
     */
    fun build(setUnmappedEndPoseRef: Reference<Pose2d>) = build().also {
        endTrajectory()
        val lastPoseField = builder::class.java.getDeclaredField("lastPoseUnmapped")
        lastPoseField.isAccessible = true
        setUnmappedEndPoseRef.set(lastPoseField.get(builder) as Pose2d)
    }

    /**
     * Build the current trajectory and add it to the current [AutonomousBunyipsOpMode] task queue with
     * the settings defined in this builder. Optionally sets a passed [setUnmappedEndPoseRef] to the last *unmapped* spliced pose.
     *
     * This method will throw an [UninitializedPropertyAccessException] if the current [BunyipsOpMode] is not an
     * [AutonomousBunyipsOpMode] or if the [AutonomousBunyipsOpMode] is not running.
     */
    @JvmOverloads
    fun addTask(setUnmappedEndPoseRef: Reference<Pose2d>? = null) {
        if (!BunyipsOpMode.isRunning || BunyipsOpMode.instance !is AutonomousBunyipsOpMode)
            throw UninitializedPropertyAccessException("Cannot call addTask() when an active AutonomousBunyipsOpMode instance is not running!")
        (BunyipsOpMode.instance as AutonomousBunyipsOpMode).add(
            priority,
            if (setUnmappedEndPoseRef != null) build(setUnmappedEndPoseRef) else build()
        )
    }
}
