package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Rotational utilities from Cartesian to Robot space.
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
     * @param cartesianXVel   the Cartesian x velocity
     * @param cartesianYVel   the Cartesian y velocity
     * @param cartesianAngVel the Cartesian clockwise heading velocity
     * @return the Robot Velocity pose representation of this Cartesian pose
     */
    @NonNull
    public static PoseVelocity2d toVel(double cartesianXVel, double cartesianYVel, double cartesianAngVel) {
        // noinspection SuspiciousNameCombination
        return new PoseVelocity2d(new Vector2d(cartesianYVel, -cartesianXVel), -cartesianAngVel);
    }

    /**
     * @param cartesianVec the Cartesian vector to convert to Robot form
     * @return the Robot vector representation of the Cartesian vector
     */
    @NonNull
    public static Vector2d toVec(@NonNull Vector2d cartesianVec) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(cartesianVec.y, -cartesianVec.x);
    }

    /**
     * @param cartesianX the Cartesian x coordinate
     * @param cartesianY the Cartesian y coordinate
     * @return the Robot vector representation of the Cartesian vector
     */
    @NonNull
    public static Vector2d toVec(double cartesianX, double cartesianY) {
        // noinspection SuspiciousNameCombination
        return new Vector2d(cartesianY, -cartesianX);
    }
}
