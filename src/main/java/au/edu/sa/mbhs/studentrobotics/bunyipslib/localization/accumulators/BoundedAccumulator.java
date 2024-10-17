package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Rect;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Field;

/**
 * Accumulator to define clamping limits on the field.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@Config
public class BoundedAccumulator extends Accumulator {
    /**
     * Maximum bounds (FTC field)
     */
    public static Rect MAX_BOUNDS = Field.MAX_BOUNDS;

    private final List<Rect> restrictedAreas = new ArrayList<>();
    private final Rect robotBoundingBox;

    /**
     * Create a new BoundedAccumulator.
     *
     * @param robotRadius the radius of the robot from the center of localization
     */
    public BoundedAccumulator(Measure<Distance> robotRadius) {
        robotBoundingBox = new Rect(new Vector2d(-robotRadius.magnitude(), -robotRadius.magnitude()),
                new Vector2d(robotRadius.magnitude(), robotRadius.magnitude()), robotRadius.unit());
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param areas the areas to restrict the robot from moving into
     * @return this
     */
    public BoundedAccumulator withRestrictedAreas(Rect... areas) {
        Collections.addAll(restrictedAreas, areas);
        return this;
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param seasonField the current season field to restrict the robot from moving into
     * @return this
     */
    public BoundedAccumulator withRestrictedAreas(Field.Season seasonField) {
        restrictedAreas.addAll(seasonField.getRestrictedAreas());
        return this;
    }

    @Override
    public void accumulate(Twist2dDual<Time> twist) {
        super.accumulate(twist);

        Pose2d currentPose = pose;
        Rect pos = robotBoundingBox.centeredAt(currentPose.position);

        Rect bounds = Rect.normalise(MAX_BOUNDS);
        if (!bounds.contains(pos)) {
            // Reset the pose to the nearest edge of the bounding box
            Vector2d newPos = new Vector2d(
                    Math.min(Math.max(currentPose.position.x, bounds.point1.x + robotBoundingBox.point2.x), bounds.point2.x - robotBoundingBox.point2.x),
                    Math.min(Math.max(currentPose.position.y, bounds.point1.y + robotBoundingBox.point2.y), bounds.point2.y - robotBoundingBox.point2.y)
            );
            currentPose = new Pose2d(newPos, currentPose.heading);
        }

        for (Rect rect : restrictedAreas) {
            Rect area = Rect.normalise(rect);
            if (area.overlaps(pos)) {
                double newX = currentPose.position.x;
                double newY = currentPose.position.y;

                // Recalculate the new position to be the nearest edge of the bounding box
                // Note: Cases where the robot is in the corner of the area perform a bit strangely as
                // they are snapped to the nearest corner of the bounding box
                if (currentPose.position.x < area.point1.x || currentPose.position.x > area.point2.x) {
                    if (currentPose.position.x < area.point1.x) {
                        newX = area.point1.x - robotBoundingBox.point2.x;
                    } else {
                        newX = area.point2.x + robotBoundingBox.point2.x;
                    }
                }

                if (currentPose.position.y < area.point1.y || currentPose.position.y > area.point2.y) {
                    if (currentPose.position.y < area.point1.y) {
                        newY = area.point1.y - robotBoundingBox.point2.y;
                    } else {
                        newY = area.point2.y + robotBoundingBox.point2.y;
                    }
                }

                // Run the new position through the bounds check again for other boxes
                currentPose = new Pose2d(new Vector2d(newX, newY), currentPose.heading);
                pos = robotBoundingBox.centeredAt(currentPose.position);
            }
        }

        pose = currentPose;
    }
}
