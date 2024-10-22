package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

/**
 * RoadRunner v1.0 logging message for a standard three deadwheel encoder localizer input.
 *
 * @since 6.0.0
 */
public final class ThreeDeadWheelInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The position and velocity of the first parallel wheel at this time.
     */
    @NonNull
    public PositionVelocityPair par0;
    /**
     * The position and velocity of the second parallel wheel at this time.
     */
    @NonNull
    public PositionVelocityPair par1;
    /**
     * The position and velocity of the perpendicular wheel at this time.
     */
    @NonNull
    public PositionVelocityPair perp;

    @SuppressWarnings("MissingJavadoc")
    public ThreeDeadWheelInputsMessage(@NonNull PositionVelocityPair par0, @NonNull PositionVelocityPair par1, @NonNull PositionVelocityPair perp) {
        timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
    }
}