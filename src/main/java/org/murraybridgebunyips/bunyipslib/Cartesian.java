package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Rotational utilities for Pose2d and Vector2d.
 *
 * @author Lucas Bubner, 2023
 */
public final class Cartesian {
    private Cartesian() {
    }

    /**
     * @param pose the pose to convert
     * @return the pose with the x and y coordinates swapped and the heading negated
     */
    public static Pose2d toPose(Pose2d pose) {
        return new Pose2d(pose.getY(), -pose.getX(), -pose.getHeading());
    }

    /**
     * @param pose the pose to convert
     * @return the pose with the x and y coordinates swapped and the heading negated
     */
    public static Pose2d fromPose(Pose2d pose) {
        return new Pose2d(-pose.getY(), pose.getX(), -pose.getHeading());
    }

    /**
     * @param x       the x coordinate
     * @param y       the y coordinate
     * @param heading the heading
     * @return the pose with the x and y coordinates swapped and the heading negated
     */
    public static Pose2d toPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(y, -x, -heading);
    }

    /**
     * @param x       the x coordinate
     * @param y       the y coordinate
     * @param heading the heading
     * @return the pose with the x and y coordinates swapped and the heading negated
     */
    public static Pose2d fromPose(double x, double y, double heading) {
        // noinspection SuspiciousNameCombination
        return new Pose2d(-y, x, -heading);
    }

    /**
     * @param vector the vector to convert
     * @return the vector with the x and y coordinates swapped
     */
    public static Vector2d toVector(Vector2d vector) {
        return new Vector2d(vector.getY(), -vector.getX());
    }

    /**
     * @param vector the vector to convert
     * @return the vector with the x and y coordinates swapped
     */
    public static Vector2d fromVector(Vector2d vector) {
        return new Vector2d(-vector.getY(), vector.getX());
    }

    /**
     * @param x the x coordinate
     * @param y the y coordinate
     * @return the vector with the x and y coordinates swapped
     */
    public static Vector2d toVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(y, -x);
    }

    /**
     * @param x the x coordinate
     * @param y the y coordinate
     * @return the vector with the x and y coordinates swapped
     */
    public static Vector2d fromVector(double x, double y) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(-y, x);
    }
}
