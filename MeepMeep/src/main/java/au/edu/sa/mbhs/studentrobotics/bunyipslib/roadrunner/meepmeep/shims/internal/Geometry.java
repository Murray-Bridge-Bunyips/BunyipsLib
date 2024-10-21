package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal;


import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Radians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance;


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
     * @param unit  the unit of both the x and y values
     * @return the vector in Inches
     */
    public static Vector2d unitVec(Vector2d value, Distance unit) {
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
    public static Pose2d unitPose(Vector2d vec, Distance tUnit, double r, Angle rUnit) {
        return new Pose2d(
                tUnit.of(vec.x).in(Inches),
                tUnit.of(vec.y).in(Inches),
                rUnit.of(r).in(Radians)
        );
    }
}
