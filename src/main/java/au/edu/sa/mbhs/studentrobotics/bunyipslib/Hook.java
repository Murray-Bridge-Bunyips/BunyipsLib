package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.sinister.loading.Preload;

/**
 * Annotation to mark a static method as a OpMode lifecycle hook, which will be executed at
 * the desired phase of any OpMode lifecycle, via the {@link BunyipsLib} listener.
 * <p>
 * Methods marked with this annotation must have <b>exactly zero method parameters</b>,
 * and <b>must be static</b>, or they will not be executed.
 * <p>
 * Method visibility does not impact the ability for this method to be run.
 * These methods are executed even if a {@link BunyipsOpMode} is not running.
 * <p>
 * To access the current OpMode from these hooks, see {@link BunyipsLib#getOpMode()}.
 * <p>
 * Do note the Hook will not execute on System OpModes or BunyipsLib-integrated OpModes (such as HardwareTester).
 * You can bypass this by passing in a boolean flag {@link #ignoreOpModeType()}, but should be used with caution.
 *
 * @author Lucas Bubner, 2024
 * @since 7.0.0
 */
@Preload
@Documented
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Hook {
    /**
     * @return the time at which this method should be executed in the OpMode lifecycle;
     * will be executed in any running OpMode, regardless of whether they are a {@link BunyipsOpMode}.
     */
    Target on();

    /**
     * @return the priority of this hook; higher value numbers are run before other hooks of lower numbers.
     * By default, hooks are assigned a priority of "level 0", running at whichever order the class loader does.
     * <p>
     * Priorities used for some integrated hooks are as follows:
     * <ol start=0>
     *     <li>General static cleanup methods</li>
     *     <li><i>Unused</i></li>
     *     <li>RobotConfig AutoInit</li>
     *     <li><i>Unused</i></li>
     *     <li>Threads Auto Stop</li>
     *     <li><i>Unused</i></li>
     *     <li>BunyipsOpMode Configuration</li>
     * </ol>
     */
    int priority() default 0;

    /**
     * Setting this flag to true will cause this hook to also execute for System and BunyipsLib-integrated OpModes.
     * <p>
     * Be advised this will run for <b>all</b> OpModes, including the default "Robot is stopped" OpMode, and system
     * events such as Sloth's <i>ProcessLoadEvent</i> and BunyipsLib's <i>ResetRobotControllerLights</i>. It is advised
     * to filter these out yourself through instance/classname checks to {@link BunyipsLib#getOpMode()}
     * if you have this flag enabled. Enabling this flag will also disable double-fire protections for POST_STOP.
     *
     * @return whether to always run this hook even on system opmodes including the default "robot is stopped" opmode.
     */
    boolean ignoreOpModeType() default false;

    /**
     * The times at which this method can be executed in the OpMode lifecycle.
     */
    enum Target {
        /**
         * To execute before the OpMode initialises, after the INIT button has been pressed.
         * <p>
         * HardwareMap and other OpMode functions are available before user code begins to use it.
         */
        PRE_INIT,
        /**
         * To execute before the OpMode starts, after the PLAY button has been pressed.
         */
        PRE_START,
        /**
         * To execute after the OpMode stops.
         */
        POST_STOP
    }
}
