package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

/**
 * Two tasks that run based on a dynamically evaluated condition.
 * <p>
 * This task wraps two tasks and when this task is initialised, the condition will be evaluated to
 * determine which task should be executing for the lifetime of this task.
 * <p>
 * Do note that this task is a task wrapper, and will discard already assigned timeout, name, priority, and subsystem information
 * to match the true or false task. Changes to these properties must be done to the child task, which will be reflected upwards.
 * Updating this wrapper will result in resets.
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
        named(conditionOnInit + " ? " + trueTask + " : " + falseTask);
        disableSubsystemAttachment = true;
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

    /**
     * Create a new conditional task with the given callbacks, task names, and condition.
     * Supplied tasks will be reset on init.
     *
     * @param onTrue              the callback to run if the condition is true
     * @param onTrueCallbackName  the task name of the onTrue callback
     * @param onFalse             the callback to run if the condition is false
     * @param onFalseCallbackName the task name of the onTrue callback
     * @param conditionOnInit     the condition to evaluate on initialisation
     */
    public ConditionalTask(@NonNull Runnable onTrue, @NonNull String onTrueCallbackName, @NonNull Runnable onFalse, @NonNull String onFalseCallbackName, @NonNull BooleanSupplier conditionOnInit) {
        this(new Lambda(onTrue).named(onTrueCallbackName), new Lambda(onFalse).named(onFalseCallbackName), conditionOnInit);
    }

    @Override
    protected void init() {
        trueTask.reset();
        falseTask.reset();
        boolean eval = conditionOnInit.getAsBoolean();
        task = eval ? trueTask : falseTask;
        task.ensureInit();
        Dbg.logd(getClass(), "initialised " + eval + ", running: " + task);
    }

    @Override
    protected void periodic() {
        if (task != null) {
            sync(task);
            task.execute();
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return task != null && task.isFinished();
    }

    @Override
    protected void onFinish() {
        if (task != null)
            task.finish();
    }

    @Override
    protected void onReset() {
        if (task != null)
            task.reset();
    }
}
