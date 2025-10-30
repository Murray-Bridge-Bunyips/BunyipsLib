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
        timeout(Arrays.stream(tasks).anyMatch(t -> t.timeout.magnitude() == 0.0) ? INFINITE_TIMEOUT :
                Seconds.of(Arrays.stream(tasks).mapToDouble(t -> t.timeout.in(Seconds)).sum()));
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
    public final void init() {
        // Match init behaviour of Sequential groups in WPILib
        currentTask.ensureInit();
    }

    @Override
    public final void periodic() {
        executeTask(currentTask);
        if (currentTask.isFinished()) {
            taskIndex++;
            if (taskIndex >= tasks.size())
                return;
            currentTask = tasks.get(taskIndex);
            // Don't waste a cycle and start the next task now
            // Matches WPILib unit tests
            currentTask.ensureInit();
        }
    }

    @Override
    protected final void onFinish() {
        currentTask.finish();
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
