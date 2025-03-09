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
        if (tasks.isEmpty()) {
            throw new IllegalArgumentException(getClass().getSimpleName() + " created with no tasks.");
        }
        StringBuilder taskNames = new StringBuilder();
        taskNames.append("[");
        taskNames.append(getClass().getSimpleName().replace("TaskGroup", ""));
        taskNames.append(": ");
        for (int i = 0; i < tasks.size() - 1; i++) {
            taskNames.append(tasks.get(i)).append(", ");
        }
        taskNames.append(tasks.get(tasks.size() - 1)).append("]");
        named(taskNames.toString());
    }

    /**
     * @return all tasks in this group
     */
    @NonNull
    public List<Task> getGroupedTasks() {
        return new ArrayList<>(tasks);
    }

    protected final void executeTask(@NonNull Task task) {
        if (task.isFinished()) {
            if (finishedTasks.add(task))
                Dbg.logd(getClass(), "sub-task (% of %, #%/max %) finished -> %s", task, this, finishedTasks.size(), tasks.size(), task.getDeltaTime().in(Seconds));
            return;
        }
        task.execute();
    }

    protected final void finishAllTasks() {
        for (Task task : tasks) {
            task.finishNow();
        }
    }

    @Override
    protected final void onFinish() {
        finishAllTasks();
    }

    @Override
    protected void onReset() {
        for (Task task : tasks) {
            task.reset();
        }
        finishedTasks.clear();
    }
}
