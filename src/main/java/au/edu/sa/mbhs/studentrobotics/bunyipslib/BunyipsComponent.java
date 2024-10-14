package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Base class for components used with a BunyipsOpMode.
 * This allows injection of the OpMode into the component, and provides a common base for all components.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public abstract class BunyipsComponent {
    // Pre-4.1.0, this property would cause a critical crash if a BunyipsComponent was instantiated as a member
    // field of the derived BOM class, however, a constructor hook is now called to supply the instance field, albeit
    // partially constructed. This partial construction won't particularly matter to the user unless they decide
    // to class-cast for some reason, in which case casting will be available on the instance during the re-assignment on runtime.
    // This makes it so a BunyipsOpMode has a brief 'partially constructed' phase that can allow components that extend
    // from this class to work properly, reducing the number of gotchas when writing OpModes.
    // This partial construction still doesn't allow conventional subsystems to be instantiated early (as hardwareMap is
    // not available), however, simple tasks that operate on functional interfaces benefit from this behaviour.

    // 5.1.0 changes: Switched the opMode field to instead be null if an instance cannot be received - this is to
    // maximise flexibility with subsystems and components of which don't have a direct dependency on a BunyipsOpMode.

    /**
     * Get a reference to the currently running {@link BunyipsOpMode}.
     * <p>
     * Will be null if this component is not running in the context of an active {@link BunyipsOpMode}.
     *
     * @see #require(BunyipsOpMode)
     * @see #opMode(Consumer)
     */
    @Nullable
    protected final BunyipsOpMode opMode = BunyipsOpMode.isRunning() ? BunyipsOpMode.getInstance() : null;

    /**
     * Null assertion for the {@link #opMode} field which throws a {@link NullPointerException} if an active {@link BunyipsOpMode}
     * is not present (i.e. the supplied field is null). This method replicates {@code Objects.requireNonNull} but has
     * a built-in message to alert the user of a non-active OpMode.
     *
     * @param nullableOpMode Nullable OpMode field
     * @return the non-null BunyipsOpMode which can be used without compiler issues
     * @throws NullPointerException if the OpMode field is null
     * @since 5.1.0
     */
    @NonNull
    protected final BunyipsOpMode require(@Nullable BunyipsOpMode nullableOpMode) throws NullPointerException {
        return Objects.requireNonNull(nullableOpMode, "This component uses a reference to BunyipsOpMode, and is not being run within the context of an active BunyipsOpMode. For this component to function properly, it must be run during the execution of a BunyipsOpMode.");
    }

    /**
     * Null check consumer for the {@link #opMode} field which will no-op the given consumer if an active {@link BunyipsOpMode}
     * is not present (i.e. the {@link #opMode} field is null). This method is the same to the {@code BunyipsOpMode.ifRunning}
     * method, and is supplied here for convenience.
     *
     * @param ifRunning the BunyipsOpMode consumer to run if the OpMode is active
     * @since 5.1.0
     */
    protected final void opMode(@NonNull Consumer<BunyipsOpMode> ifRunning) {
        if (opMode != null)
            ifRunning.accept(opMode);
    }
}
