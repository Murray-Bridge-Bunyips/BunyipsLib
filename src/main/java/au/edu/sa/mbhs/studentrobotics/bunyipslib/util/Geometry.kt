package au.edu.sa.mbhs.studentrobotics.bunyipslib.util

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.approx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.lerp
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.moveTowards
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.radToDeg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.smoothDamp
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapDeltaRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import dev.frozenmilk.util.cell.Cell
import kotlin.math.hypot

/**
 * Defines useful conversion methods for RoadRunner geometry types, restoring some removed methods from RoadRunner v0.5.
 *
 * This class was created for migration purposes between RoadRunner v0.5 and v1.0, where new types were introduced
 * and some methods removed, but these methods are still useful within BunyipsLib.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
object Geometry {
    /**
     * Create a new [Pose2d] with zero values.
     *
     * @return the zero pose
     */
    @JvmStatic
    fun zeroPose(): Pose2d {
        return Pose2d(0.0, 0.0, 0.0)
    }

    /**
     * Create a new [PoseVelocity2d] with zero values.
     *
     * @return the zero pose velocity
     */
    @JvmStatic
    fun zeroVel(): PoseVelocity2d {
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }

    /**
     * Create a new [Vector2d] with zero values.
     *
     * @return the zero vector
     */
    @JvmStatic
    fun zeroVec(): Vector2d {
        return Vector2d(0.0, 0.0)
    }

    /**
     * Create a new [Twist2d] with zero values.
     *
     * @return the zero twist
     */
    @JvmStatic
    fun zeroTwist(): Twist2d {
        return Twist2d(Vector2d(0.0, 0.0), 0.0)
    }

    /**
     * Create a new [PoseVelocity2d] with x, y, and heading values.
     *
     * @param xVel   the +forward velocity
     * @param yVel   the +left velocity
     * @param angVel the +anticlockwise heading velocity
     * @return the pose velocity
     */
    @JvmStatic
    fun vel(xVel: Double, yVel: Double, angVel: Double): PoseVelocity2d {
        return PoseVelocity2d(Vector2d(xVel, yVel), angVel)
    }

    /**
     * Create a vector in the desired units to be converted to a conventional Inches unit.
     *
     * @param unit  the unit of both the x and y values
     * @return the vector in Inches
     */
    @JvmStatic
    infix fun Vector2d.inchesFrom(unit: Distance): Vector2d {
        return Vector2d(unit of this.x to Inches, unit of this.y to Inches)
    }

    /**
     * Create a pose in the desired units to be converted to conventional Inches and Radians units.
     *
     * @param tUnit the unit of both the x and y values
     * @param r     the heading value
     * @param rUnit the unit of the heading value
     * @return the vector in Inches
     */
    @JvmStatic
    @JvmOverloads
    fun Vector2d.poseFrom(tUnit: Distance = Inches, r: Double, rUnit: Angle): Pose2d {
        return Pose2d(
            tUnit of this.x to Inches,
            tUnit of this.y to Inches,
            rUnit of r to Radians
        )
    }

    /**
     * Compare two poses with an epsilon value.
     *
     * @param other the other pose
     * @return whether the two poses are equal within the epsilon value 1e-6
     */
    @JvmStatic
    infix fun Pose2d.approx(other: Pose2d): Boolean {
        return this.position.x approx other.position.x
                && this.position.y approx other.position.y
                && this.heading.toDouble() approx other.heading.toDouble()
    }

    /**
     * Compare two poses with an epsilon value plus a renormalisation of heading.
     *
     * @param other the other pose
     * @return whether the two poses are equal within the epsilon value 1e-6 and heading radius
     */
    @JvmStatic
    infix fun Pose2d.approxNormHeading(other: Pose2d): Boolean {
        return this.position.x approx other.position.x
                && this.position.y approx other.position.y
                && (this.heading.toDouble() - other.heading.toDouble()).wrapDeltaRadians() approx 0
    }

    /**
     * Compare two vectors with an epsilon value.
     *
     * @param other the other vector to compare
     * @return whether the two vectors are equal within the epsilon value 1e-6
     */
    @JvmStatic
    infix fun Vector2d.approx(other: Vector2d): Boolean {
        return this.x approx other.x && this.y approx other.y
    }

    /**
     * Calculate the distance between two vectors.
     *
     * @param other the other vector
     * @return the distance between the two vectors
     */
    @JvmStatic
    infix fun Vector2d.distTo(other: Vector2d): Double {
        return hypot(other.x - this.x, other.y - this.y)
    }

    /**
     * Returns a user-friendly representation of a [Pose2d].
     *
     * @return the user-friendly string
     */
    @JvmStatic
    fun Pose2d.toUserString(): String {
        return "Pose2d(x=%.1f, y=%.1f, r=%.1f°)".format(
            this.position.x,
            this.position.y,
            Math.toDegrees(this.heading.toDouble())
        )
    }

    /**
     * Returns a user-friendly representation of a [PoseVelocity2d].
     *
     * @return the user-friendly string
     */
    @JvmStatic
    fun PoseVelocity2d.toUserString(): String {
        return "PoseVelocity2d(x=%.1f, y=%.1f, r=%.1f°)".format(
            this.linearVel.x,
            this.linearVel.y,
            Math.toDegrees(this.angVel)
        )
    }

    /**
     * Linearly interpolate between two vectors.
     * The [t] parameter is not clamped between 0-1.
     */
    @JvmStatic
    fun Vector2d.lerp(other: Vector2d, t: Double): Vector2d {
        return Vector2d(this.x + (other.x - this.x) * t, this.y + (other.y - this.y) * t)
    }

    /**
     * Linearly interpolate between two poses.
     * The [t] parameter is not clamped between 0-1.
     */
    @JvmStatic
    fun Pose2d.lerp(other: Pose2d, t: Double): Pose2d {
        return Pose2d(
            this.position.lerp(other.position, t),
            this.heading.lerp(other.heading, t)
        )
    }

    /**
     * Linearly interpolate between two rotations.
     * The [t] parameter is not clamped between 0-1.
     * Rotation value will take the shortest path.
     */
    @JvmStatic
    fun Rotation2d.lerp(other: Rotation2d, t: Double): Rotation2d {
        return Rotation2d.exp((Radians.of(this.log()) to Radians.of(other.log()) lerp t to Radians))
    }

    /**
     * Move a vector towards another vector by a step size.
     */
    @JvmStatic
    fun Vector2d.moveTowards(other: Vector2d, stepSizeInches: Double): Vector2d {
        return Vector2d(
            this.x.moveTowards(other.x, stepSizeInches),
            this.y.moveTowards(other.y, stepSizeInches)
        )
    }

    /**
     * Move a pose towards another pose by a step size.
     */
    @JvmStatic
    fun Pose2d.moveTowards(other: Pose2d, stepSizeInches: Double, stepSizeRad: Double): Pose2d {
        return Pose2d(
            this.position.moveTowards(other.position, stepSizeInches),
            this.heading.moveTowards(other.heading, Radians.of(stepSizeRad))
        )
    }

    /**
     * Move a rotation towards another rotation by a step size.
     */
    @JvmStatic
    fun Rotation2d.moveTowards(other: Rotation2d, stepSize: Measure<Angle>): Rotation2d {
        return Rotation2d.exp((Radians.of(this.log()).moveTowards(Radians.of(other.log()), stepSize) to Radians))
    }

    /**
     * Smoothly damp a vector towards another vector.
     */
    @JvmStatic
    fun Vector2d.smoothDamp(
        target: Vector2d,
        currentVelocity: Cell<Double>,
        smoothTime: Measure<Time>,
        maxVelocity: Number,
        deltaTime: Measure<Time>
    ): Vector2d {
        return Vector2d(
            this.x.smoothDamp(target.x, currentVelocity, smoothTime, maxVelocity.toDouble(), deltaTime),
            this.y.smoothDamp(target.y, currentVelocity, smoothTime, maxVelocity.toDouble(), deltaTime)
        )
    }

    /**
     * Smoothly damp a pose towards another pose.
     * Velocity will be in degrees per second.
     */
    @JvmStatic
    fun Pose2d.smoothDamp(
        target: Pose2d,
        currentVelocity: Cell<Double>,
        smoothTime: Measure<Time>,
        maxVelocity: Number,
        deltaTime: Measure<Time>
    ): Pose2d {
        return Pose2d(
            this.position.smoothDamp(target.position, currentVelocity, smoothTime, maxVelocity.toDouble(), deltaTime),
            this.heading.smoothDamp(target.heading, currentVelocity, smoothTime, maxVelocity.toDouble(), deltaTime)
        )
    }

    /**
     * Smoothly damp a rotation towards another rotation.
     * Velocity will be in degrees per second.
     */
    @JvmStatic
    fun Rotation2d.smoothDamp(
        target: Rotation2d,
        currentVelocity: Cell<Double>,
        smoothTime: Measure<Time>,
        maxVelocity: Number,
        deltaTime: Measure<Time>
    ): Rotation2d {
        return Rotation2d.exp(
            Degrees.of(this.log().radToDeg()).smoothDamp(
                Degrees.of(target.log().radToDeg()),
                currentVelocity,
                smoothTime,
                maxVelocity.toDouble(),
                deltaTime
            ) to Radians
        )
    }

    /**
     * Maps a [Pose2d] into another pose applied with a [PoseMap].
     */
    @JvmStatic
    fun Pose2d.map(poseMap: PoseMap) = poseMap.map(Pose2dDual.constant(this, 1)).value()
}
