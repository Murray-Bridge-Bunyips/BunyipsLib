package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Two tasks that run based on a dynamically evaluated condition.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class ConditionalTask extends Task {
    private final Task trueTask;
    private final Task falseTask;
    private final BooleanSupplier condition;

    /**
     * Create a new conditional task with the given tasks and condition.
     *
     * @param trueTask  the task to run if the condition is true
     * @param falseTask the task to run if the condition is false
     * @param condition the condition to evaluate
     */
    public ConditionalTask(@NonNull Task trueTask, @NonNull Task falseTask, @NonNull BooleanSupplier condition) {
        this.trueTask = trueTask;
        this.falseTask = falseTask;
        this.condition = condition;
        withName("Conditional " + trueTask + " / " + falseTask);
    }

    /**
     * Create a new conditional task with the given callbacks and condition.
     *
     * @param onTrue    the callback to run if the condition is true
     * @param onFalse   the callback to run if the condition is false
     * @param condition the condition to evaluate
     */
    public ConditionalTask(@NonNull Runnable onTrue, @NonNull Runnable onFalse, @NonNull BooleanSupplier condition) {
        this(new RunTask(onTrue), new RunTask(onFalse), condition);
    }

    @Override
    protected void init() {
        if (condition.getAsBoolean()) {
            trueTask.run();
        } else {
            falseTask.run();
        }
    }

    @Override
    protected void periodic() {
        if (condition.getAsBoolean()) {
            trueTask.run();
        } else {
            falseTask.run();
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return trueTask.pollFinished() || falseTask.pollFinished();
    }

    @Override
    protected void onReset() {
        trueTask.reset();
        falseTask.reset();
    }
}
