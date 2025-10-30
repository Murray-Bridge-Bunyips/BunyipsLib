package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

/**
 * A group of tasks.
 * <p>
 * Users must be careful to ensure they do not allocate tasks that use the same subsystems when
 * running in parallel (all task groups except SequentialTaskGroup), otherwise tasks will try
 * to run on the same subsystems and cause some to get cancelled due to the subsystem being busy.
 * <p>
 * While task groups wrap tasks, they only update timeouts and names on construction to represent the static state
 * of tasks supplied at construction. They do not update if any of the internal tasks adjust timeout, but will be
 * still caught by the finish checks, not affecting functionality. Incrementing groups are the only exception
 * where it is timeout-dependent.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public abstract class TaskGroup extends Task {
    protected final ArrayList<Task> tasks = new ArrayList<>();
    private final HashSet<Task> finishedTasks = new HashSet<>();

    protected TaskGroup() {
        disableSubsystemAttachment = true;
    }

    protected void setTasks(@NonNull List<Task> tasks) {
        this.tasks.addAll(tasks);
        if (tasks.isEmpty())
            throw new IllegalArgumentException(getClass().getSimpleName() + " created with no tasks.");
        StringBuilder taskNames = new StringBuilder();
        taskNames.append("[");
        taskNames.append(getClass().getSimpleName().replace("TaskGroup", ""));
        taskNames.append(": ");
        for (int i = 0; i < tasks.size() - 1; i++) {
            Task t = tasks.get(i);
            taskNames.append(t).append(", ");
            if (t.isPriority)
                asPriority();
        }
        taskNames.append(tasks.get(tasks.size() - 1)).append("]");
        // Names are only updated on construction and will not reflect changes to the composed tasks
        // Timeout definitions are treated similarly depending on the implementation of the TaskGroup
        named(taskNames.toString());
    }

    /**
     * @return all copy of all tasks in this group
     */
    @NonNull
    public List<Task> getGroupedTasks() {
        return new ArrayList<>(tasks);
    }

    protected final void executeTask(@NonNull Task task) {
        // We don't call sync as the name will not be updated, task groups will not be attached to subsystems,
        // and timeouts depend on the internal implementation.
        if (task.isPriority) isPriority = true;
        else if (isPriority) task.isPriority = true;
        task.execute();
        if (task.isFinished() && finishedTasks.add(task))
            Dbg.logd(getClass(), "sub-task (% of %, #%/max %) finished -> %s", task, this, finishedTasks.size(), tasks.size(), task.getElapsedTime().in(Seconds));
    }

    protected final void finishAllTasks() {
        for (Task task : tasks)
            task.finish();
    }

    @Override
    protected void onFinish() {
        finishAllTasks();
    }

    @Override
    protected void onReset() {
        for (Task task : tasks)
            task.reset();
        finishedTasks.clear();
    }
}
