package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;

/**
 * Defines useful conversion methods for RoadRunner geometry types, including conversion between {@link Pose2d} and {@link PoseVelocity2d},
 * as well as restoring some removed methods from RoadRunner v0.5.
 * <p>
 * This class was created for migration purposes between RoadRunner v0.5 and v1.0, where new types were introduced
 * and some methods removed, but these methods are still useful within BunyipsLib.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class Geometry {
    private Geometry() {
    }

    /**
     * Create a new {@link Pose2d} with zero values.
     *
     * @return the zero pose
     */
    public static Pose2d zeroPose() {
        return new Pose2d(0, 0, 0);
    }

    /**
     * Create a new {@link PoseVelocity2d} with zero values.
     *
     * @return the zero pose velocity
     */
    public static PoseVelocity2d zeroVel() {
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    /**
     * Create a new {@link Vector2d} with zero values.
     *
     * @return the zero vector
     */
    public static Vector2d zeroVec() {
        return new Vector2d(0, 0);
    }

    /**
     * Create a new {@link Twist2d} with zero values.
     *
     * @return the zero twist
     */
    public static Twist2d zeroTwist() {
        return new Twist2d(new Vector2d(0, 0), 0);
    }

    /**
     * Create a vector in the desired units to be converted to a conventional Inches unit.
     *
     * @param value the vector
     * @param unit the unit of both the x and y values
     * @return the vector in Inches
     */
    public static Vector2d unitVec(Vector2d value, Distance unit) {
        return new Vector2d(unit.of(value.x).in(Inches), unit.of(value.y).in(Inches));
    }

    /**
     * Create a pose in the desired units to be converted to conventional Inches and Radians units.
     *
     * @param value the pose
     * @param tUnit the unit of both the x and y values
     * @param rUnit the unit of the heading value
     * @return the pose in corresponding Inches and Radians units
     */
    public static Pose2d unitPose(Pose2d value, Distance tUnit, Angle rUnit) {
        return new Pose2d(
                tUnit.of(value.position.x).in(Inches),
                tUnit.of(value.position.y).in(Inches),
                rUnit.of(value.heading.toDouble()).in(Radians)
        );
    }

    /**
     * Convert a {@link Pose2d} to a {@link PoseVelocity2d}.
     *
     * @param pose the pose to convert
     * @return the converted pose
     */
    public static PoseVelocity2d poseToVel(Pose2d pose) {
        return new PoseVelocity2d(pose.position, pose.heading.toDouble());
    }

    /**
     * Convert a {@link PoseVelocity2d} to a {@link Pose2d}.
     *
     * @param pose the pose to convert
     * @return the converted pose
     */
    public static Pose2d poseFromVel(PoseVelocity2d pose) {
        return new Pose2d(pose.linearVel, pose.angVel);
    }

    /**
     * Compare two poses with an epsilon value.
     *
     * @param a the first pose
     * @param b the second pose
     * @return whether the two poses are equal within the epsilon value 1e-6
     */
    public static boolean epsilonEquals(Pose2d a, Pose2d b) {
        return Mathf.approximatelyEquals(a.position.x, b.position.x)
                && Mathf.approximatelyEquals(a.position.y, b.position.y)
                && Mathf.approximatelyEquals(a.heading.toDouble(), b.heading.toDouble());
    }

    /**
     * Compare two poses with an epsilon value plus a renormalisation of heading.
     *
     * @param a the first pose
     * @param b the second pose
     * @return whether the two poses are equal within the epsilon value 1e-6 and heading radius
     */
    public static boolean epsilonEqualsHeading(Pose2d a, Pose2d b) {
        return Mathf.approximatelyEquals(a.position.x, b.position.x)
                && Mathf.approximatelyEquals(a.position.y, b.position.y)
                && Mathf.approximatelyEquals(Mathf.radianModulus(a.heading.toDouble() - b.heading.toDouble()), 0);
    }

    /**
     * Compare two vectors with an epsilon value.
     *
     * @param a the first vector
     * @param b the second vector
     * @return whether the two vectors are equal within the epsilon value 1e-6
     */
    public static boolean epsilonEquals(Vector2d a, Vector2d b) {
        return Mathf.approximatelyEquals(a.x, b.x)
                && Mathf.approximatelyEquals(a.y, b.y);
    }

    /**
     * Calculate the distance between two vectors.
     *
     * @param a the first vector
     * @param b the second vector
     * @return the distance between the two vectors
     */
    public static double distBetween(Vector2d a, Vector2d b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }
}
