package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import androidx.annotation.Nullable;

import org.murraybridgebunyips.bunyipslib.tasks.IdleTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;

/**
 * Base class for all robot subsystems.
 * Integrates with the Task system to allow for task-based command scheduling.
 *
 * @author Lucas Bubner, 2024
 * @see Scheduler
 */
public abstract class BunyipsSubsystem extends BunyipsComponent {
    private final ArrayList<Integer> dependencies = new ArrayList<>();
    private Task currentTask;
    private Task defaultTask = new IdleTask();
    private boolean mutedReports;
    private boolean shouldRun = true;

    /**
     * Utility function to run NullSafety.assertComponentArgs() on the given parameters, usually on
     * the motors/hardware passed into the constructor. If this check fails, your subsystem
     * will automatically disable the update() method from calling to prevent exceptions, no-oping
     * the subsystem. A COM_FAULT will be added to telemetry, and exceptions from this class will be muted.
     * @param parameters constructor parameters for your subsystem that should be checked for null,
     *                   in which case the subsystem should be disabled
     */
    public void assertParamsNotNull(Object... parameters) {
        // If a previous check has already failed, we don't need to check again otherwise we might
        // erase a previous check that failed
        if (!shouldRun) return;
        // assertComponentArgs will manage telemetry/impl of errors being ignored, all we need to do
        // is check if it failed and if so, disable the subsystem
        shouldRun = NullSafety.assertComponentArgs(getClass(), parameters);
        if (!shouldRun) {
            Dbg.error(getClass(), "Subsystem has been disabled as assertParamsNotNull() failed.");
        }
    }

    /**
     * Get the current task for this subsystem.
     * If the current task is null or finished, the default task will be returned.
     *
     * @return The current task, null if the subsytem is disabled
     */
    @Nullable
    public Task getCurrentTask() {
        if (!shouldRun) return null;
        if (currentTask == null || currentTask.isFinished()) {
            currentTask = defaultTask;
        }
        return currentTask;
    }

    /**
     * Call to mute the Scheduler from reporting task status for this subsystem.
     * Useful for subsystems that are not task-based.
     */
    public void muteTaskReports() {
        mutedReports = true;
    }

    /**
     * Set the default task for this subsystem, which will be run when no other task is running.
     *
     * @param defaultTask The task to set as the default task
     */
    public final void setDefaultTask(Task defaultTask) {
        if (!shouldRun) return;
        this.defaultTask = defaultTask;
    }

    /**
     * Set the current task to the given task.
     *
     * @param newTask The task to set as the current task
     * @return whether the task was successfully set or ignored
     */
    public final boolean setCurrentTask(Task newTask) {
        if (!shouldRun) {
            Dbg.warn(getClass(), "Subsystem is disabled from failed assertion, ignoring task change.");
            return false;
        }
        if (currentTask == newTask)
            return true;

        // Lockout if a task is currently running that is not the default task
        if (currentTask != defaultTask) {
            // Override if the task is designed to override
            // shouldOverrideOnConflict might be null if it is a non-command task
            if (Boolean.TRUE.equals(newTask.shouldOverrideOnConflict())) {
                setHighPriorityCurrentTask(newTask);
                return true;
            }
            Dbg.log(getClass(), "Ignored task change: %->%", currentTask.getName(), newTask.getName());
            return false;
        }

        newTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (currentTask == defaultTask)
            defaultTask.onFinish();
        Dbg.logd(getClass(), "Task changed: %->%", currentTask.getName(), newTask.getName());
        currentTask = newTask;
        return true;
    }

    /**
     * Add a dependency from another task to this subsystem.
     *
     * @param taskHashCode The hash code of the task to add as a dependency
     */
    public final void addDependencyFromTask(int taskHashCode) {
        dependencies.add(taskHashCode);
    }

    /**
     * Get the dependencies for this subsystem.
     *
     * @return The dependencies for this subsystem
     */
    public final ArrayList<Integer> getTaskDependencies() {
        return dependencies;
    }

    /**
     * Set the current task to the given task, overriding any current task.
     *
     * @param currentTask The task to set as the current task
     */
    public final void setHighPriorityCurrentTask(Task currentTask) {
        if (!shouldRun) {
            Dbg.warn(getClass(), "Subsystem is disabled from failed assertion, ignoring high-priority task change.");
            return;
        }
        // Task will be cancelled abruptly, run the finish callback now
        if (this.currentTask != defaultTask) {
            Dbg.warn(getClass(), "Task changed: %(INT)->%", this.currentTask.getName(), currentTask.getName());
            this.currentTask.forceFinish();
        }
        currentTask.reset();
        // Default task technically can't finish, but it can be interrupted, so we will just run the finish callback
        if (this.currentTask == defaultTask)
            defaultTask.onFinish();
        this.currentTask = currentTask;
    }

    /**
     * Update the subsystem and run the current task, if tasks are not set up this will just call {@link #periodic()}.
     * This method should be called if you are running this subsystem manually, otherwise it will be called by the Scheduler.
     */
    public final void update() {
        if (!shouldRun) return;
        Task task = getCurrentTask();
        if (task != null) {
            if (task == defaultTask && defaultTask.pollFinished()) {
                throw new EmergencyStop("Default task should never finish!");
            }
            task.run();
            // Update the state of isFinished() after running the task as it may have changed
            task.pollFinished();
            if (!mutedReports) {
                Scheduler.addSubsystemTaskReport(
                        getClass().getSimpleName(),
                        task.getName(),
                        round(task.getDeltaTime(), 1)
                );
            }
        }
        // This should be the only place where periodic() is called for this subsystem
        periodic();
    }

    /**
     * To be updated periodically on every hardware loop.
     * This method should not be called manually, and should only be called from the context
     * of the {@link #update()} method.
     * @see #update()
     */
    protected abstract void periodic();
}
