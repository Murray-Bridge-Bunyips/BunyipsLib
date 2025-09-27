package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.OTOSLocalizer;

/**
 * RoadRunner v1.0 logging message for the initialisation of the OTOS localizer.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class OTOSLocalizerParamsMessage {
    /**
     * Used parameters for localization.
     */
    public OTOSLocalizer.Params params;
    /**
     * Whether the OTOS is reported on the I2C bus.
     */
    public boolean isConnected;
    /**
     * Self test check result.
     */
    public boolean selfTestPassed;
    /**
     * Reported hardware version.
     */
    public SparkFunOTOS.Version hardwareVersion;
    /**
     * Reported firmware version.
     */
    public SparkFunOTOS.Version firmwareVersion;

    @SuppressWarnings("MissingJavadoc")
    public OTOSLocalizerParamsMessage(@NonNull OTOSLocalizer.Params params, boolean isConnected, boolean selfTestPassed, SparkFunOTOS.Version hardwareVersion, SparkFunOTOS.Version firmwareVersion) {
        this.params = params;
        this.isConnected = isConnected;
        this.selfTestPassed = selfTestPassed;
        this.hardwareVersion = hardwareVersion;
        this.firmwareVersion = firmwareVersion;
    }
}

