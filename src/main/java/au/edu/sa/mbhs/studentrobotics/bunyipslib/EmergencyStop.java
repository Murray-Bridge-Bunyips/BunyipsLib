package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

/**
 * Custom exception to be thrown when BunyipsLib should end the OpMode following a critical error.
 * This ensures the {@link Exceptions} handler will be called but also allows the OpMode to be ended immediately.
 * <p>
 * To ensure no code can run and to immediately transition to stop with no stacktrace,
 * throw the {@link OpModeManagerImpl.ForceStopException} or call the relevant terminate method.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class EmergencyStop extends RuntimeException {
    /**
     * Emergency stop and bypass the {@link Exceptions} handler.
     *
     * @param message the message to display on the Driver Station.
     */
    public EmergencyStop(@NonNull String message) {
        super(message);
    }

    /**
     * Emergency stop and bypass the {@link Exceptions} handler.
     *
     * @param message the message to display on the Driver Station.
     * @param cause the cause to affix to this exception, if any.
     */
    public EmergencyStop(@NonNull String message, @Nullable Throwable cause) {
        super(message, cause);
    }
}
