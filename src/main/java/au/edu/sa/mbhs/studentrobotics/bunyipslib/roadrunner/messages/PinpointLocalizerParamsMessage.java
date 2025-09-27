package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.PinpointLocalizer;

/**
 * RoadRunner v1.0 logging message for the initialisation of the Pinpoint localizer.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class PinpointLocalizerParamsMessage {
    /**
     * Used parameters for localization.
     */
    public PinpointLocalizer.Params params;
    /**
     * Device ID of the Pinpoint. Should be 1.
     */
    public int deviceId;
    /**
     * Firmware version of the Pinpoint.
     */
    public int deviceVersion;
    /**
     * Scaling yaw factor used by the Pinpoint;
     */
    public float yawScalar;

    @SuppressWarnings("MissingJavadoc")
    public PinpointLocalizerParamsMessage(@NonNull PinpointLocalizer.Params params, int deviceId, int deviceVersion, float yawScalar) {
        this.params = params;
        this.deviceId = deviceId;
        this.deviceVersion = deviceVersion;
        this.yawScalar = yawScalar;
    }
}

