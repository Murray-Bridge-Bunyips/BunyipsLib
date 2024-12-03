package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs one after the other, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class SequentialTaskGroup extends TaskGroup {
    private int taskIndex;
    private Task currentTask;

    /**
     * Create a new SequentialTaskGroup with tasks.
     *
     * @param tasks The tasks to run in sequence
     */
    public SequentialTaskGroup(@NonNull Task... tasks) {
        // Timeout will be represented by the sum of all tasks, unless one is infinite then the entire group is infinite.
        // This works for a sequential following where each task gets its own runtime
        timeout(Arrays.stream(tasks).anyMatch(t -> t.getTimeout().magnitude() == 0.0) ? INFINITE_TIMEOUT :
                Seconds.of(Arrays.stream(tasks).mapToDouble(t -> t.getTimeout().in(Seconds)).sum()));
        setTasks(Arrays.asList(tasks));
        currentTask = this.tasks.get(0);
    }

    /**
     * Create a new SequentialTaskGroup with tasks.
     *
     * @param tasks The tasks to run in sequence
     */
    public SequentialTaskGroup(@NonNull List<Task> tasks) {
        this(tasks.toArray(new Task[0]));
    }

    @Override
    public final void periodic() {
        if (currentTask.poll()) {
            taskIndex++;
            if (taskIndex >= tasks.size()) {
                finish();
                return;
            }
            currentTask = tasks.get(taskIndex);
        } else {
            executeTask(currentTask);
            named(currentTask.toString());
        }
    }

    @Override
    protected final void onReset() {
        super.onReset();
        taskIndex = 0;
        currentTask = tasks.get(0);
    }

    @Override
    protected boolean isTaskFinished() {
        return taskIndex >= tasks.size() && currentTask.isFinished();
    }
}
