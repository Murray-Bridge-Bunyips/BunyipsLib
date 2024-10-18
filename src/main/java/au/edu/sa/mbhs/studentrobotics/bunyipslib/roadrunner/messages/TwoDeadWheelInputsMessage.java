package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * RoadRunner v1.0 logging message for a standard two deadwheel encoder localizer input.
 *
 * @since 6.0.0
 */
public final class TwoDeadWheelInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The position and velocity of the parallel wheel at this time.
     */
    public PositionVelocityPair par;
    /**
     * The position and velocity of the perpendicular wheel at this time.
     */
    public PositionVelocityPair perp;
    /**
     * The yaw angle of the IMU at this time.
     */
    public double yaw;
    /**
     * The pitch angle of the IMU at this time.
     */
    public double pitch;
    /**
     * The roll angle of the IMU at this time.
     */
    public double roll;
    /**
     * The x rotation rate of the IMU at this time.
     */
    public double xRotationRate;
    /**
     * The y rotation rate of the IMU at this time.
     */
    public double yRotationRate;
    /**
     * The z rotation rate of the IMU at this time.
     */
    public double zRotationRate;

    @SuppressWarnings("MissingJavadoc")
    public TwoDeadWheelInputsMessage(PositionVelocityPair par, PositionVelocityPair perp, YawPitchRollAngles angles, AngularVelocity angularVelocity) {
        timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
        {
            yaw = angles.getYaw(AngleUnit.RADIANS);
            pitch = angles.getPitch(AngleUnit.RADIANS);
            roll = angles.getRoll(AngleUnit.RADIANS);
        }
        {
            xRotationRate = angularVelocity.xRotationRate;
            yRotationRate = angularVelocity.yRotationRate;
            zRotationRate = angularVelocity.zRotationRate;
        }
    }
}
