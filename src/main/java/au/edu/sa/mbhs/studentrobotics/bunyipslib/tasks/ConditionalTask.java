package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

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
        withName("Conditional " + trueTask + " / " + falseTask);
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
        this(new RunTask(onTrue), new RunTask(onFalse), conditionOnInit);
    }

    @Override
    protected void init() {
        trueTask.reset();
        falseTask.reset();
        task = conditionOnInit.getAsBoolean() ? trueTask : falseTask;
        task.run();
        withTimeout(task.getTimeout());
    }

    @Override
    protected void periodic() {
        task.run();
    }

    @Override
    protected boolean isTaskFinished() {
        return task.pollFinished();
    }

    @Override
    protected void onReset() {
        task.reset();
    }
}
