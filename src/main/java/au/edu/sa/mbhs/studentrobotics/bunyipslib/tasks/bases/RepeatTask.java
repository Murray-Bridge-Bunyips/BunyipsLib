package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

/**
 * A task that will auto-reset and repeat itself after it is completed.
 * <p>
 * Do note that this task is a task wrapper, and will discard already assigned timeout, name, priority, and subsystem information
 * to match the repeated task. Changes to these properties must be done to the child task, which will be reflected upwards.
 * Updating this wrapper will result in resets.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class RepeatTask extends Task {
    private final Task task;

    /**
     * Create a new RepeatTask with the given task.
     *
     * @param task The task to repeat after it is completed.
     */
    public RepeatTask(@NonNull Task task) {
        this.task = task;
        named(task + " (rep.)");
        disableSubsystemAttachment = true;
    }

    @Override
    protected void periodic() {
        named(task + " (rep.)");
        // Infinite timeout as it will be repeated
        task.isPriority = isPriority;
        task.execute();
        if (task.isFinished())
            task.reset();
    }

    @Override
    protected void onFinish() {
        task.finish();
    }

    @Override
    protected void onReset() {
        task.reset();
    }
}
