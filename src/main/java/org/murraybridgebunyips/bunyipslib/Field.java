package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.Arrays;
import java.util.List;

/**
 * Utilities related the FTC field and its components.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public final class Field {
    /**
     * A rect representing the maximum bounds of the FTC field.
     */
    public static final Rect MAX_BOUNDS = new Rect(new Vector2d(-72, -72), new Vector2d(72, 72), Inches);

    private Field() {
    }

    /**
     * Enum representing different season fields.
     */
    public enum Season {
        /**
         * 2024-2025 season field.
         */
        INTO_THE_DEEP {
            @Override
            public List<Rect> getRestrictedAreas() {
                return Arrays.asList(
                        // Submersible zone
                        new Rect(new Vector2d(14.75, 22.375), new Vector2d(-14.75, -22.375), Inches),
                        // Extrusions from the submersible zone
                        new Rect(new Vector2d(24, 24.5), new Vector2d(14.5, 23.5), Inches),
                        new Rect(new Vector2d(-24, 24.5), new Vector2d(-14.5, 23.5), Inches),
                        new Rect(new Vector2d(24, -24.5), new Vector2d(14.5, -23.5), Inches),
                        new Rect(new Vector2d(-24, -24.5), new Vector2d(-14.5, -23.5), Inches)
                );
            }

            @Override
            public AprilTagLibrary getAprilTagLibrary() {
                return AprilTagGameDatabase.getIntoTheDeepTagLibrary();
            }
        };

        /**
         * @return areas on the field that cannot be entered via method of drivetrain localization (such as obstacles)
         */
        public abstract List<Rect> getRestrictedAreas();

        /**
         * @return the AprilTag library for this season field
         */
        public abstract AprilTagLibrary getAprilTagLibrary();
    }
}
