package au.edu.sa.mbhs.studentrobotics.bunyipslib.util

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.approx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapDeltaRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.hypot

/**
 * Defines useful conversion methods for RoadRunner geometry types, restoring some removed methods from RoadRunner v0.5.
 *
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
        return "Pose2d(x=" + this.position.x + ", y=" + this.position.y + ", r=" + Math.toDegrees(this.heading.toDouble()) + "°)"
    }
}
