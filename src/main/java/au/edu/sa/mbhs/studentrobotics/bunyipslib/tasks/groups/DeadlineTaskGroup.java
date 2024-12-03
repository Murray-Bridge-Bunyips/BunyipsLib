package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until the first queued task is finished.
 * Once this task is finished, all other tasks are cancelled if not completed.
 * The task that will be the decider for the deadline is the first task passed as an argument.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class DeadlineTaskGroup extends TaskGroup {
    /**
     * Create a new DeadlineTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public DeadlineTaskGroup(@NonNull Task... tasks) {
        // Timeout is defined by the very first task
        timeout(tasks[0].getTimeout());
        setTasks(Arrays.asList(tasks));
    }

    /**
     * Create a new DeadlineTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public DeadlineTaskGroup(@NonNull List<Task> tasks) {
        this(tasks.toArray(new Task[0]));
    }

    @Override
    public final void periodic() {
        for (Task task : tasks) {
            executeTask(task);
        }
    }

    @Override
    public final boolean isTaskFinished() {
        Task firstTask = tasks.get(0);
        boolean firstTaskFinished = firstTask != null && firstTask.poll();
        if (firstTaskFinished) {
            finishAllTasks();
        }
        return firstTaskFinished;
    }
}
