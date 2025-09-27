package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/**
 * RoadRunner v1.0 logging message for the OTOS localizer.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class OTOSLocalizerInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * Reported status with warnings.
     */
    public SparkFunOTOS.Status status;
    /**
     * Measured acceleration.
     */
    public SparkFunOTOS.Pose2D acceleration;
    /**
     * Standard deviation of position.
     */
    public SparkFunOTOS.Pose2D positionStdDev;
    /**
     * Standard deviation of velocity.
     */
    public SparkFunOTOS.Pose2D velocityStdDev;
    /**
     * Standard deviation of acceleration.
     */
    public SparkFunOTOS.Pose2D accelerationStdDev;

    @SuppressWarnings("MissingJavadoc")
    public OTOSLocalizerInputsMessage(SparkFunOTOS.Status status, SparkFunOTOS.Pose2D acceleration, SparkFunOTOS.Pose2D positionStdDev, SparkFunOTOS.Pose2D velocityStdDev, SparkFunOTOS.Pose2D accelerationStdDev) {
        timestamp = System.nanoTime();
        this.status = status;
        this.acceleration = acceleration;
        this.positionStdDev = positionStdDev;
        this.velocityStdDev = velocityStdDev;
        this.accelerationStdDev = accelerationStdDev;
    }
}

