package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A group of tasks that runs all at once, until they are all finished.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class ParallelTaskGroup extends TaskGroup {
    /**
     * Create a new ParallelTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public ParallelTaskGroup(@NonNull Task... tasks) {
        // Try to extract the highest timeout to be the timeout of this task group, however if one is infinite
        // then the group is infinite. This works for parallel applications as every task will go till completion.
        timeout(Arrays.stream(tasks).anyMatch(t -> t.timeout.magnitude() == 0.0) ? INFINITE_TIMEOUT :
                Seconds.of(Arrays.stream(tasks).mapToDouble(t -> t.timeout.in(Seconds)).max().orElse(0.0)));
        setTasks(Arrays.asList(tasks));
    }

    /**
     * Create a new ParallelTaskGroup with tasks.
     *
     * @param tasks The tasks to run together
     */
    public ParallelTaskGroup(@NonNull List<Task> tasks) {
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
        for (Task task : tasks) {
            if (!task.isFinished()) return false;
        }
        return true;
    }
}
