package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;

/**
 * Rotational utilities for Pose2d and Vector2d, including conversion from Cartesian to Robot space.
 * <p>
 * The Cartesian Coordinate System is a 2D coordinate system where the x-axis goes to the right (pos X) and the y-axis goes up/forward (pos Y).
 * Angles on the Cartesian Coordinate System are measured clockwise from the positive y-axis.
 * <p>
 * The Robot Coordinate System is a 2D coordinate system where the x-axis goes up/forward (pos X) and the y-axis goes left (pos Y), and
 * the angles are measured anti-clockwise from the positive x-axis. This is the coordinate system used by the RoadRunner library,
 * and perfectly models a <a href="https://upload.wikimedia.org/wikipedia/commons/thumb/4/4c/Unit_circle_angles_color.svg/1024px-Unit_circle_angles_color.svg.png">Unit Circle</a>.
 *
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
public final class Cartesian {
    private Cartesian() {
    }

    /**
     * Rotate a Cartesian vector.
     *
     * @param cartesianVec the vector to rotate, must be in a Cartesian vector
     * @param rot          the angle to rotate these points around by
     * @return the rotated Cartesian pose
     */
    public static Vector2d rotate(Vector2d cartesianVec, Measure<Angle> rot) {
        double t = rot.in(Radians);
        // 2D rotation matrix
        // | cos(t) -sin(t) | | x | = | x cos(t) - y sin(t) |
        // | sin(t)  cos(t) | | y | = | x sin(t) + y cos(t) |
        return new Vector2d(
                cartesianVec.x * Math.cos(t) - cartesianVec.y * Math.sin(t),
                cartesianVec.x * Math.sin(t) + cartesianVec.y * Math.cos(t)
        );
    }

    /**
     * @param pose the Cartesian pose to convert to Robot form
     * @return the Robot pose representation of the Cartesian pose
     */
    public static Pose2d toPose(Pose2d pose) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(pose.position.y, -pose.position.x, -pose.heading.toDouble());
    }

    /**
     * @param pose the Robot pose to convert to Cartesian form
     * @return the Cartesian pose representation of the Robot pose
     */
    public static Pose2d fromPose(Pose2d pose) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(-pose.position.y, pose.position.x, -pose.heading.toDouble());
    }

    /**
     * @param x       the Cartesian x coordinate
     * @param y       the Cartesian y coordinate
     * @param heading the Cartesian clockwise heading
     * @return the Robot pose representation of the Cartesian pose
     */
    public static Pose2d toPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(y, -x, -heading);
    }

    /**
     * @param x       the Robot x coordinate
     * @param y       the Robot y coordinate
     * @param heading the Robot anti-clockwise heading
     * @return the Cartesian pose representation of the Robot pose
     */
    public static Pose2d fromPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(-y, x, -heading);
    }

    /**
     * @param vector the Cartesian vector to convert to Robot form
     * @return the Robot vector representation of the Cartesian vector
     */
    public static Vector2d toVector(Vector2d vector) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(vector.y, -vector.x);
    }

    /**
     * @param vector the Robot vector to convert to Cartesian form
     * @return the Cartesian vector representation of the Robot vector
     */
    public static Vector2d fromVector(Vector2d vector) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(-vector.y, vector.x);
    }

    /**
     * @param x the Cartesian x coordinate
     * @param y the Cartesian y coordinate
     * @return the Robot vector representation of the Cartesian vector
     */
    public static Vector2d toVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(y, -x);
    }

    /**
     * @param x the Robot x coordinate
     * @param y the Robot y coordinate
     * @return the Cartesian vector representation of the Robot vector
     */
    public static Vector2d fromVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(-y, x);
    }
}
