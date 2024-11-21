package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Direction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingPositions;

/**
 * Util for obtaining the backdrop ID based on a Spike Mark position and field side.
 *
 * @author Lucas Bubner, 2024
 * @since 3.0.0
 */
public final class SpikeMarkBackdropId {
    private SpikeMarkBackdropId() {
    }

    /**
     * Get the backdrop ID based on the Spike Mark position and field side.
     *
     * @param spikeMarkPosition     the position of the Spike Mark as detected by vision
     * @param robotStartingPosition the side of the field the robot is starting on
     * @return the backdrop ID, -1 if invalid arguments
     */
    public static int get(@NonNull Direction spikeMarkPosition, @NonNull StartingPositions robotStartingPosition) {
        return switch (robotStartingPosition) {
            case STARTING_RED_LEFT, STARTING_RED_RIGHT -> switch (spikeMarkPosition) {
                case LEFT -> 4;
                case FORWARD -> 5;
                case RIGHT -> 6;
                default -> -1;
            };
            case STARTING_BLUE_LEFT, STARTING_BLUE_RIGHT -> switch (spikeMarkPosition) {
                case LEFT -> 1;
                case FORWARD -> 2;
                case RIGHT -> 3;
                default -> -1;
            };
        };
    }
}
