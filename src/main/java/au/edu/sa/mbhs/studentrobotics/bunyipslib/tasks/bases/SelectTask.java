package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Select a task to run based on a hashmap of states and a supplier of states. States are observed periodically,
 * and when a state change is observed, the current task is finished, reset, and the new task is initialised immediately.
 * <p>
 * Do note that this task is a task wrapper, and will discard already assigned timeout, name, priority, and subsystem information
 * to match the selected task. Changes to these properties must be done to the child task, which will be reflected upwards.
 * Updating this wrapper will result in resets.
 *
 * @param <T> the type of the state
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class SelectTask<T> extends Task {
    private final Supplier<T> stateSupplier;
    private final HashMap<T, Task> tasks = new HashMap<>();
    private Task currentTask;

    /**
     * Create a new select task with the given state supplier.
     *
     * @param stateSupplier the supplier of states to check in the {@link #when} method
     */
    public SelectTask(@NonNull Supplier<T> stateSupplier) {
        this.stateSupplier = stateSupplier;
        named("Task (sel.)");
        disableSubsystemAttachment = true;
    }

    /**
     * Add a task to run when the state is equal to the given state.
     * <p>
     * For Kotlin users, calling this method can be done with the notation {@code `when`}
     * (see <a href="https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin">here</a>),
     * or by calling the alias {@code on}.
     *
     * @param state the state to run the task on
     * @param task  the task to run
     * @return this task
     */
    @NonNull
    @SuppressLint("NoHardKeywords")
    public SelectTask<T> when(T state, @NonNull Task task) {
        tasks.put(state, task);
        return this;
    }

    /**
     * Add a task to run when the state is equal to the given state.
     *
     * @param state the state to run the task on
     * @param task  the task to run
     * @return this task
     */
    @NonNull
    public SelectTask<T> on(T state, @NonNull Task task) {
        return when(state, task);
    }

    @Override
    protected void init() {
        currentTask = tasks.get(stateSupplier.get());
        if (currentTask != null)
            currentTask.ensureInit();
    }

    @Override
    protected void periodic() {
        Task task = tasks.get(stateSupplier.get());
        if (task != null && task != currentTask) {
            currentTask.finish();
            currentTask.reset();
            currentTask = task;
        }
        if (currentTask == null) return;
        sync(currentTask);
        currentTask.execute();
    }

    @Override
    protected boolean isTaskFinished() {
        return currentTask != null && currentTask.isFinished();
    }

    @Override
    protected void onFinish() {
        for (Task task : tasks.values())
            task.finish();
    }

    @Override
    protected void onReset() {
        for (Task task : tasks.values())
            task.reset();
    }
}
