package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * RoadRunner v1.0 logging message for the Pinpoint localizer.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public final class PinpointLocalizerInputsMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * Raw encoder reading of the parallel pod.
     */
    public int encoderX;
    /**
     * Raw encoder reading of the perpendicular pod.
     */
    public int encoderY;
    /**
     * Unnormalized heading of the robot in degrees.
     */
    public double degUnnormalized;
    /**
     * Reported status of the Pinpoint.
     */
    public GoBildaPinpointDriver.DeviceStatus status;
    /**
     * Report update frequency of the Pinpoint in Hertz.
     */
    public double frequencyHz;

    @SuppressWarnings("MissingJavadoc")
    public PinpointLocalizerInputsMessage(int encoderX, int encoderY, double degUnnormalized, GoBildaPinpointDriver.DeviceStatus status, double frequencyHz) {
        timestamp = System.nanoTime();
        this.encoderX = encoderX;
        this.encoderY = encoderY;
        this.degUnnormalized = degUnnormalized;
        this.status = status;
        this.frequencyHz = frequencyHz;
    }
}

