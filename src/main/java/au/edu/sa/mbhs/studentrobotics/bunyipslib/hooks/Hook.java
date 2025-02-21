package au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import dev.frozenmilk.sinister.Preload;

/**
 * Annotation to mark a static method as a OpMode lifecycle hook, which will be executed at
 * the desired phase of the OpMode lifecycle, via the {@link BunyipsLib} listener.
 * <p>
 * Methods marked with this annotation must have <b>exactly zero method parameters</b>,
 * and <b>must be static</b>, or they will not be executed.
 * <p>
 * Method visibility does not impact the ability for this method to be run.
 * These methods are executed even if a {@link BunyipsOpMode} is not running.
 * <p>
 * To access the current OpMode from these hooks, see {@link BunyipsLib#getOpMode()}.
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
     * Priorities used for integrated hooks are as follows:
     * <ol start=0>
     *     <li>Static cleanup methods</li>
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
