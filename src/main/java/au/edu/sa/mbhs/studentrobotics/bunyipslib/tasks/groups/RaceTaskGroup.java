package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

import java.util.Arrays;

/**
 * A group of tasks that runs all at once, until one of them is finished.
 * Once a single task is finished, all other tasks are cancelled.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class RaceTaskGroup extends TaskGroup {
    /**
     * Create a new RaceTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public RaceTaskGroup(Task... tasks) {
        // Try to extract the lowest timeout to be the timeout of this task group.
        // This works for a race where the first task to finish will finish the rest.
        super(Seconds.of(Arrays.stream(tasks).mapToDouble(t -> t.getTimeout().in(Seconds)).min().orElse(0.0)), tasks);
    }

    @Override
    public final void periodic() {
        for (Task task : tasks) {
            executeTask(task);
        }
    }

    @Override
    public final boolean isTaskFinished() {
        for (Task task : tasks) {
            if (task.pollFinished()) {
                finishAllTasks();
                return true;
            }
        }
        return false;
    }
}
