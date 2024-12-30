package au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.sinister.Preload;

/**
 * Annotation to mark a static method as a cleanup method, which will be executed at the end of the OpMode lifecycle,
 * via the {@link BunyipsLib} listener.
 * <p>
 * Methods marked with this annotation must have <b>exactly zero method parameters</b>,
 * and <b>must be static</b>, or they will not be executed.
 * <p>
 * Method visibility does not impact the ability for this method to be run.
 *
 * @author Lucas Bubner, 2024
 * @since 6.2.0
 */
@Preload
@Documented
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Cleanup {
}
