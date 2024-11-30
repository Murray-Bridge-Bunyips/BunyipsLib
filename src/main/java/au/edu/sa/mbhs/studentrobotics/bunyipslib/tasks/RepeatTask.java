package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A task that will auto-reset and repeat itself after it is completed.
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
        withName(task + " (Repeat)");
    }

    @Override
    protected void init() {
        task.run();
    }

    @Override
    protected void periodic() {
        if (task.pollFinished()) {
            task.reset();
        }
        task.run();
    }

    @Override
    protected void onFinish() {
        task.finishNow();
    }

    @Override
    protected boolean isTaskFinished() {
        return false;
    }
}
