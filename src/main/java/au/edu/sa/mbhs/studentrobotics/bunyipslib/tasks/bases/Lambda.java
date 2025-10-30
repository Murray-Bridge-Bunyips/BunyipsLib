package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.NonNull;

import java.util.function.Consumer;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

/**
 * A task to run a callback before immediately completing.
 * <p>
 * {@code new Lambda(() -> telemetry.add("Hello world"));}
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class Lambda extends Task {
    // This value may need adjustment in combination with a SequentialTaskGroup where the timeouts are summed,
    // however we can't tell how long a single method will execute for so we need to assume Lambdas will only
    // do one small task.
    /**
     * The epsilon value for the Lambda timeout.
     */
    public static int EPSILON_MS = 300;
    private final Consumer<Task> callback;

    /**
     * Run the given callback immediately.
     *
     * @param callback The callback to run, taking in an instance of the current task
     */
    public Lambda(@NonNull Consumer<Task> callback) {
        // For Lambdas, we can't have an infinite timeout, but we can use a very short one instead.
        // This is so the schedulers do not mistake this task as for one that will end up running forever,
        // as all Lambdas will run only once. This also helps telemetry decide how long a task will execute for.
        timeout = Milliseconds.of(EPSILON_MS);
        this.callback = callback;
        named("Callback");
    }

    /**
     * Run the given callback immediately.
     *
     * @param callback The callback to run
     */
    public Lambda(@NonNull Runnable callback) {
        this((t) -> callback.run());
    }

    // Kotlin constructor

    /**
     * Run the given callback immediately.
     *
     * @param callback The callback to run
     */
    public Lambda(@NonNull Function0<Unit> callback) {
        this((t) -> callback.invoke());
    }

    /**
     * Run nothing and complete, this is useful as a placeholder task.
     */
    public Lambda() {
        this(() -> {
        });
        named("No-op");
    }

    @Override
    protected void init() {
        callback.accept(this);
    }

    @Override
    protected final boolean isTaskFinished() {
        // Lambdas may sometimes have their timeouts adjusted at runtime
        return timeout.lte(Milliseconds.of(EPSILON_MS));
    }
}
