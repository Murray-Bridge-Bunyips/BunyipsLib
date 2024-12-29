package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.BooleanSupplier;

/**
 * Two tasks that run based on a dynamically evaluated condition.
 * <p>
 * This task wraps two tasks and when this task is initialised, the condition will be evaluated to
 * determine which task should be executing for the lifetime of this task.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class ConditionalTask extends Task {
    private final Task trueTask;
    private final Task falseTask;
    private final BooleanSupplier conditionOnInit;
    @Nullable
    private Task task;

    /**
     * Create a new conditional task with the given tasks and condition.
     * Supplied tasks will be reset on init.
     *
     * @param trueTask        the task to run if the condition is true
     * @param falseTask       the task to run if the condition is false
     * @param conditionOnInit the condition to evaluate on initialisation
     */
    public ConditionalTask(@NonNull Task trueTask, @NonNull Task falseTask, @NonNull BooleanSupplier conditionOnInit) {
        this.trueTask = trueTask;
        this.falseTask = falseTask;
        this.conditionOnInit = conditionOnInit;
        named("Conditional " + trueTask + " / " + falseTask);
    }

    /**
     * Create a new conditional task with the given callbacks and condition.
     * Supplied tasks will be reset on init.
     *
     * @param onTrue          the callback to run if the condition is true
     * @param onFalse         the callback to run if the condition is false
     * @param conditionOnInit the condition to evaluate on initialisation
     */
    public ConditionalTask(@NonNull Runnable onTrue, @NonNull Runnable onFalse, @NonNull BooleanSupplier conditionOnInit) {
        this(new Lambda(onTrue), new Lambda(onFalse), conditionOnInit);
    }

    @Override
    protected void init() {
        trueTask.reset();
        falseTask.reset();
        task = conditionOnInit.getAsBoolean() ? trueTask : falseTask;
        task.execute();
        timeout = task.timeout;
    }

    @Override
    protected void periodic() {
        if (task != null)
            task.execute();
    }

    @Override
    protected boolean isTaskFinished() {
        return task != null && task.poll();
    }

    @Override
    protected void onReset() {
        if (task != null)
            task.reset();
    }
}
