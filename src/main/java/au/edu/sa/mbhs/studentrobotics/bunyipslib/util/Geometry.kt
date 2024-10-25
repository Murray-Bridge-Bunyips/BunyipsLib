package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;

/**
 * Defines useful conversion methods for RoadRunner geometry types, restoring some removed methods from RoadRunner v0.5.
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
    @NonNull
    public static Pose2d zeroPose() {
        return new Pose2d(0, 0, 0);
    }

    /**
     * Create a new {@link PoseVelocity2d} with zero values.
     *
     * @return the zero pose velocity
     */
    @NonNull
    public static PoseVelocity2d zeroVel() {
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    /**
     * Create a new {@link Vector2d} with zero values.
     *
     * @return the zero vector
     */
    @NonNull
    public static Vector2d zeroVec() {
        return new Vector2d(0, 0);
    }

    /**
     * Create a new {@link Twist2d} with zero values.
     *
     * @return the zero twist
     */
    @NonNull
    public static Twist2d zeroTwist() {
        return new Twist2d(new Vector2d(0, 0), 0);
    }

    /**
     * Create a new {@link PoseVelocity2d} with x, y, and heading values.
     *
     * @param xVel   the +forward velocity
     * @param yVel   the +left velocity
     * @param angVel the +anticlockwise heading velocity
     * @return the pose velocity
     */
    @NonNull
    public static PoseVelocity2d vel(double xVel, double yVel, double angVel) {
        return new PoseVelocity2d(new Vector2d(xVel, yVel), angVel);
    }

    /**
     * Create a vector in the desired units to be converted to a conventional Inches unit.
     *
     * @param value the vector
     * @param unit  the unit of both the x and y values
     * @return the vector in Inches
     */
    @NonNull
    public static Vector2d unitVec(@NonNull Vector2d value, @NonNull Distance unit) {
        return new Vector2d(unit.of(value.x).in(Inches), unit.of(value.y).in(Inches));
    }

    /**
     * Create a pose in the desired units to be converted to conventional Inches and Radians units.
     *
     * @param vec   the vector element
     * @param tUnit the unit of both the x and y values
     * @param r     the heading value
     * @param rUnit the unit of the heading value
     * @return the vector in Inches
     */
    @NonNull
    public static Pose2d unitPose(@NonNull Vector2d vec, @NonNull Distance tUnit, double r, @NonNull Angle rUnit) {
        return new Pose2d(
                tUnit.of(vec.x).in(Inches),
                tUnit.of(vec.y).in(Inches),
                rUnit.of(r).in(Radians)
        );
    }

    /**
     * Compare two poses with an epsilon value.
     *
     * @param a the first pose
     * @param b the second pose
     * @return whether the two poses are equal within the epsilon value 1e-6
     */
    public static boolean epsilonEquals(@NonNull Pose2d a, @NonNull Pose2d b) {
        return Mathf.approx(a.position.x, b.position.x)
                && Mathf.approx(a.position.y, b.position.y)
                && Mathf.approx(a.heading.toDouble(), b.heading.toDouble());
    }

    /**
     * Compare two poses with an epsilon value plus a renormalisation of heading.
     *
     * @param a the first pose
     * @param b the second pose
     * @return whether the two poses are equal within the epsilon value 1e-6 and heading radius
     */
    public static boolean epsilonEqualsHeading(@NonNull Pose2d a, @NonNull Pose2d b) {
        return Mathf.approx(a.position.x, b.position.x)
                && Mathf.approx(a.position.y, b.position.y)
                && Mathf.approx(Mathf.wrapDeltaRadians(a.heading.toDouble() - b.heading.toDouble()), 0);
    }

    /**
     * Compare two vectors with an epsilon value.
     *
     * @param a the first vector
     * @param b the second vector
     * @return whether the two vectors are equal within the epsilon value 1e-6
     */
    public static boolean epsilonEquals(@NonNull Vector2d a, @NonNull Vector2d b) {
        return Mathf.approx(a.x, b.x)
                && Mathf.approx(a.y, b.y);
    }

    /**
     * Calculate the distance between two vectors.
     *
     * @param a the first vector
     * @param b the second vector
     * @return the distance between the two vectors
     */
    public static double distBetween(@NonNull Vector2d a, @NonNull Vector2d b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }

    /**
     * Returns a user-friendly representation of a {@link Pose2d}.
     *
     * @param pose the pose to convert to a user-friendly (heading in degrees) string
     * @return the user-friendly string
     */
    @NonNull
    public static String toString(@NonNull Pose2d pose) {
        return "Pose2d(x=" + pose.position.x + ", y=" + pose.position.y + ", r=" + Math.toDegrees(pose.heading.toDouble()) + "°)";
    }
}
