package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * RoadRunner v1.0 logging message for a standard Mecanum localizer input.
 *
 * @since 6.0.0
 */
public final class MecanumLocalizerInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The position and velocity of the left front wheel at this time.
     */
    @NonNull
    public PositionVelocityPair leftFront;
    /**
     * The position and velocity of the left back wheel at this time.
     */
    @NonNull
    public PositionVelocityPair leftBack;
    /**
     * The position and velocity of the right back wheel at this time.
     */
    @NonNull
    public PositionVelocityPair rightBack;
    /**
     * The position and velocity of the right front wheel at this time.
     */
    @NonNull
    public PositionVelocityPair rightFront;
    /**
     * The yaw angle at this time.
     */
    public double yaw;
    /**
     * The pitch angle at this time.
     */
    public double pitch;
    /**
     * The roll angle at this time.
     */
    public double roll;

    @SuppressWarnings("MissingJavadoc")
    public MecanumLocalizerInputsMessage(@NonNull PositionVelocityPair leftFront, @NonNull PositionVelocityPair leftBack, @NonNull PositionVelocityPair rightBack, @NonNull PositionVelocityPair rightFront, @NonNull YawPitchRollAngles angles) {
        timestamp = System.nanoTime();
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
        yaw = angles.getYaw(AngleUnit.RADIANS);
        pitch = angles.getPitch(AngleUnit.RADIANS);
        roll = angles.getRoll(AngleUnit.RADIANS);
    }
}
