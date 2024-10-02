package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Field;
import org.murraybridgebunyips.bunyipslib.Rect;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Runnable component to define localization clamping limits on the field.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
@Config
public class BoundedLocalization implements Runnable {
    /**
     * Maximum bounds (FTC field)
     */
    public static Rect MAX_BOUNDS = Field.MAX_BOUNDS;

    private final List<Rect> restrictedAreas = new ArrayList<>();
    private final Rect robotBoundingBox;
    private final Supplier<Pose2d> getPose;
    private final Consumer<Pose2d> setPose;

    /**
     * Create a new BoundedLocalization runner.
     *
     * @param robotRadius the radius of the robot from the center of localization
     * @param getPose     supplier that supplies the current pose of the robot
     * @param setPose     consumer that will update the current pose of the robot
     */
    public BoundedLocalization(Measure<Distance> robotRadius, Supplier<Pose2d> getPose, Consumer<Pose2d> setPose) {
        robotBoundingBox = new Rect(new Vector2d(-robotRadius.magnitude(), -robotRadius.magnitude()),
                new Vector2d(robotRadius.magnitude(), robotRadius.magnitude()), robotRadius.unit());
        this.getPose = getPose;
        this.setPose = setPose;
        BunyipsOpMode.ifRunning(opMode -> {
            opMode.onActiveLoop(this);
            Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode.");
        });
    }

    /**
     * Create a new BoundedLocalization runner.
     *
     * @param robotRadius the radius of the robot from the center of localization
     * @param localizer   the localizer to use for pose setting and updating
     */
    public BoundedLocalization(Measure<Distance> robotRadius, Localizer localizer) {
        this(robotRadius, localizer::getPoseEstimate, localizer::setPoseEstimate);
    }

    /**
     * Create a new BoundedLocalization runner.
     *
     * @param robotRadius the radius of the robot from the center of localization
     * @param getPose     supplier that supplies the current pose of the robot
     * @param setPose     consumer that will update the current pose of the robot
     * @return the new BoundedLocalization runner
     */
    public static BoundedLocalization enable(Measure<Distance> robotRadius, Supplier<Pose2d> getPose, Consumer<Pose2d> setPose) {
        return new BoundedLocalization(robotRadius, getPose, setPose);
    }

    /**
     * Create a new BoundedLocalization runner.
     *
     * @param robotRadius the radius of the robot from the center of localization
     * @param localizer   the localizer to use for pose setting and updating
     * @return the new BoundedLocalization runner
     */
    public static BoundedLocalization enable(Measure<Distance> robotRadius, Localizer localizer) {
        return new BoundedLocalization(robotRadius, localizer);
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param areas the areas to restrict the robot from moving into
     * @return this
     */
    public BoundedLocalization setRestrictedAreas(Rect... areas) {
        Collections.addAll(restrictedAreas, areas);
        return this;
    }

    /**
     * Set areas on the field that the robot should not be able to move into.
     *
     * @param seasonField the current season field to restrict the robot from moving into
     * @return this
     */
    public BoundedLocalization setRestrictedAreas(Field.Season seasonField) {
        restrictedAreas.addAll(seasonField.getRestrictedAreas());
        return this;
    }

    @Override
    public void run() {
        Pose2d pose = getPose.get();
        Rect pos = robotBoundingBox.centeredAt(pose.vec());

        Rect bounds = Rect.normalise(MAX_BOUNDS);
        if (!bounds.contains(pos)) {
            // Reset the pose to the nearest edge of the bounding box
            Vector2d newPos = new Vector2d(
                    Math.min(Math.max(pose.getX(), bounds.point1.getX() + robotBoundingBox.point2.getX()), bounds.point2.getX() - robotBoundingBox.point2.getX()),
                    Math.min(Math.max(pose.getY(), bounds.point1.getY() + robotBoundingBox.point2.getY()), bounds.point2.getY() - robotBoundingBox.point2.getY())
            );
            pose = new Pose2d(newPos, pose.getHeading());
        }

        for (Rect rect : restrictedAreas) {
            Rect area = Rect.normalise(rect);
            if (area.overlaps(pos)) {
                double newX = pose.getX();
                double newY = pose.getY();

                // Recalculate the new position to be the nearest edge of the bounding box
                // Note: Cases where the robot is in the corner of the area perform a bit strangely as
                // they are snapped to the nearest corner of the bounding box
                if (pose.getX() < area.point1.getX() || pose.getX() > area.point2.getX()) {
                    if (pose.getX() < area.point1.getX()) {
                        newX = area.point1.getX() - robotBoundingBox.point2.getX();
                    } else {
                        newX = area.point2.getX() + robotBoundingBox.point2.getX();
                    }
                }

                if (pose.getY() < area.point1.getY() || pose.getY() > area.point2.getY()) {
                    if (pose.getY() < area.point1.getY()) {
                        newY = area.point1.getY() - robotBoundingBox.point2.getY();
                    } else {
                        newY = area.point2.getY() + robotBoundingBox.point2.getY();
                    }
                }

                // Run the new position through the bounds check again for other boxes
                pose = new Pose2d(new Vector2d(newX, newY), pose.getHeading());
                pos = robotBoundingBox.centeredAt(pose.vec());
            }
        }

        setPose.accept(pose);
    }
}
