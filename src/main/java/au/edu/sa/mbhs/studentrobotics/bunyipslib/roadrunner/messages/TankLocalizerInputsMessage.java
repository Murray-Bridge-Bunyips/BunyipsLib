package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import java.util.List;

/**
 * RoadRunner v1.0 logging message for a standard Tank localizer input.
 *
 * @since 6.0.0
 */
public final class TankLocalizerInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The position and velocity of the left wheels at this time.
     */
    public PositionVelocityPair[] left;
    /**
     * The position and velocity of the right wheels at this time.
     */
    public PositionVelocityPair[] right;

    @SuppressWarnings("MissingJavadoc")
    public TankLocalizerInputsMessage(List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
