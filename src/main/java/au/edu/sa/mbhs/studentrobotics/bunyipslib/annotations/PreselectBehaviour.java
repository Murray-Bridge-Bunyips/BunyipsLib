package au.edu.sa.mbhs.studentrobotics.bunyipslib.annotations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Declares how an {@link Autonomous} OpMode's {@link Autonomous#preselectTeleOp()} is executed.
 * <p>
 * By default, the {@code preselectTeleOp} is simply selected on the Driver Station when the Auto OpMode ends. This annotation
 * can be attached in combination with the Autonomous annotation with preselect in order to also initialise the OpMode
 * and optionally start it after a set delay (default to 8 seconds to match with the standard FTC Auto-TeleOp interval),
 * completely automatically to aid the drive team.
 * <p>
 * This annotation must be attached to an OpMode annotated with {@link Autonomous}, and the
 * {@link Autonomous#preselectTeleOp()} string must be non-empty and valid. This annotation otherwise does nothing.
 *
 * @author Lucas Bubner, 2025
 * @since 8.1.0
 */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface PreselectBehaviour {
    /**
     * When a preselected TeleOp is loaded, this action will immediately occur after it is selected on the Driver Station.
     * <p>
     * {@link Action#INIT} will initialise the OpMode and do nothing else.
     * <p>
     * {@link Action#START} will initialise the OpMode, wait {@link #startDelaySec()} seconds, and start the OpMode.
     *
     * @return the action to execute
     */
    Action action();

    /**
     * When using {@link Action#START}, a delay of this many seconds will be induced in the interval between initialising
     * and playing the OpMode.
     * <p>
     * This delay defaults to 8 seconds which is the standard FTC Auto-TeleOp interval length. You may adjust this value
     * but be aware of implications of starting an OpMode early and possibly moving actuators in the Auto-TeleOp interval.
     * <p>
     * This delay is ignored if {@link Action#INIT} is used.
     *
     * @return the delay to wait between INIT and START in seconds
     */
    long startDelaySec() default 8;

    /**
     * The enhanced action to take for the preselected TeleOp.
     */
    enum Action {
        /**
         * Automatically presses the INIT button on the preselected TeleOp OpMode and does nothing else.
         */
        INIT,
        /**
         * Automatically presses the INIT button on the preselected TeleOp OpMode, waits {@link #startDelaySec()} seconds, and presses the PLAY button.
         */
        START
    }
}
