package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Select a task to run based on a hashmap of states and a supplier of states.
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
    protected void periodic() {
        Task task = tasks.get(stateSupplier.get());
        if (task != null && task != currentTask) {
            currentTask.finishNow();
            currentTask = task;
        }
        if (currentTask == null) return;
        named(currentTask.toString());
        timeout = currentTask.timeout;
        currentTask.isPriority = isPriority;
        currentTask.execute();
    }

    @Override
    protected boolean isTaskFinished() {
        Task task = tasks.get(stateSupplier.get());
        return task != null && task.poll();
    }

    @Override
    protected void onFinish() {
        for (Task task : tasks.values()) {
            task.finishNow();
        }
    }

    @Override
    protected void onReset() {
        for (Task task : tasks.values()) {
            task.reset();
        }
    }
}
