package org.murraybridgebunyips.bunyipslib;

/**
 * Custom exception to be thrown when BunyipsLib should end the OpMode following a critical error.
 * This ensures the Exceptions handler will be called but also allows the OpMode to be ended immediately.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class EmergencyStop extends RuntimeException {
    /**
     * Emergency stop a BunyipsOpMode.
     *
     * @param message the message to display on the Driver Station.
     */
    public EmergencyStop(String message) {
        super(message);
    }
}
