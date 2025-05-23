package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Diff;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Rect;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Field;

/**
 * Accumulator to define clamping limits on the field.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class BoundedAccumulator extends Accumulator {
    /**
     * Maximum bounds (default full FTC field)
     */
    @NonNull
    public static Rect MAX_BOUNDS = Field.MAX_BOUNDS;
    /**
     * Time in seconds that will be used to inhibit the velocity calculation after the robot pose is rebounded.
     * This is to sustain the velocity calculation for a short period of time as the rebound is not called every loop.
     */
    public static double MANUAL_VELOCITY_INHIBITION_TIME = 0.5;

    private final List<Rect> restrictedAreas = new ArrayList<>();
    private final Rect robotBoundingBox;
    private final ElapsedTime velocityInhibitionTimer = new ElapsedTime();
    private final Diff xVel = new Diff();
    private final Diff yVel = new Diff();

    /**
     * Create a new BoundedAccumulator.
     *
     * @param robotRadius the radius of the robot from the center of localization,
     *                    will be interpreted as a square bounding box
     */
    public BoundedAccumulator(@NonNull Measure<Distance> robotRadius) {
        robotBoundingBox = new Rect(new Vector2d(-robotRadius.magnitude(), -robotRadius.magnitude()),
                new Vector2d(robotRadius.magnitude(), robotRadius.magnitude()), robotRadius.unit());
        Dashboard.enableConfig(getClass());
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param areas the areas to restrict the robot from moving into
     * @return this
     */
    @NonNull
    public BoundedAccumulator withRestrictedAreas(@NonNull Rect... areas) {
        Collections.addAll(restrictedAreas, areas);
        return this;
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param seasonField the current season field to restrict the robot from moving into
     * @return this
     */
    @NonNull
    public BoundedAccumulator withRestrictedAreas(@NonNull Field.Season seasonField) {
        restrictedAreas.addAll(seasonField.getRestrictedAreas());
        return this;
    }

    @Override
    public void accumulate(@NonNull Twist2dDual<Time> twist) {
        super.accumulate(twist);

        Pose2d currentPose = pose;
        // Note we don't consider heading for a possible intersections at an angle, but it has enough accuracy as-is
        Rect pos = robotBoundingBox.centeredAt(currentPose.position);

        Rect bounds = Rect.normalise(MAX_BOUNDS);
        if (!bounds.contains(pos)) {
            // Reset the pose to the nearest edge of the bounding box
            Vector2d newPos = new Vector2d(
                    Math.min(Math.max(currentPose.position.x, bounds.point1.x + robotBoundingBox.point2.x), bounds.point2.x - robotBoundingBox.point2.x),
                    Math.min(Math.max(currentPose.position.y, bounds.point1.y + robotBoundingBox.point2.y), bounds.point2.y - robotBoundingBox.point2.y)
            );
            currentPose = new Pose2d(newPos, currentPose.heading);
            velocityInhibitionTimer.reset();
        }

        for (Rect rect : restrictedAreas) {
            Rect area = Rect.normalise(rect);
            if (area.overlaps(pos)) {
                double newX = currentPose.position.x;
                double newY = currentPose.position.y;

                // Calculate how much overlap the two positions have
                double overlapX = Math.min(pos.point2.x - area.point1.x, area.point2.x - pos.point1.x);
                double overlapY = Math.min(pos.point2.y - area.point1.y, area.point2.y - pos.point1.y);

                double robotCenterX = (pos.point1.x + pos.point2.x) / 2.0;
                double robotCenterY = (pos.point1.y + pos.point2.y) / 2.0;
                double areaCenterX = (area.point1.x + area.point2.x) / 2.0;
                double areaCenterY = (area.point1.y + area.point2.y) / 2.0;

                // Decide which way to "push" the robot back out of the overlapping box
                double correctionX = robotCenterX < areaCenterX ? -overlapX : overlapX;
                double correctionY = robotCenterY < areaCenterY ? -overlapY : overlapY;

                // Prefer one axis to let the robot slide along an axis not affected by the bounding box
                if (Math.abs(correctionX) < Math.abs(correctionY)) {
                    newX = currentPose.position.x + correctionX;
                } else if (Math.abs(correctionY) < Math.abs(correctionX)) {
                    newY = currentPose.position.y + correctionY;
                } else {
                    newX = currentPose.position.x + correctionX;
                    newY = currentPose.position.y + correctionY;
                }

                // Run the new position through the bounds check again for other boxes
                currentPose = new Pose2d(new Vector2d(newX, newY), currentPose.heading);
                pos = robotBoundingBox.centeredAt(currentPose.position);
                velocityInhibitionTimer.reset();
            }
        }

        // Switch to manual velocity calculation based on the adjusted pose
        // We latch the inhibition for some time due to the non-constant nature of the rebounding calculation
        if (velocityInhibitionTimer.seconds() < MANUAL_VELOCITY_INHIBITION_TIME) {
            velocity = new PoseVelocity2d(
                    new Vector2d(xVel.apply(currentPose.position.x), yVel.apply(currentPose.position.y)),
                    twist.velocity().value().angVel
            );
        }

        pose = currentPose;
    }
}
