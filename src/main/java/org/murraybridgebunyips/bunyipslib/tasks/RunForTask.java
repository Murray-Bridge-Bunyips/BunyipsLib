package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A task to run for a timeout.
 * <p>
 * {@code new RunForTask(5, () -> arm.spin());}
 */
public class RunForTask extends Task {
    private final Runnable callback;

    /**
     * Subsystem-independent task.
     *
     * @param timeout  The time to run the task for
     * @param callback The callback to run every loop
     */
    public RunForTask(double timeout, Runnable callback) {
        super(timeout);
        this.callback = callback;
    }

    /**
     * Subsystem-dependent task.
     *
     * @param timeout                        The time to run the task for
     * @param callback                       The callback to run every loop
     * @param dependency                     The subsystem to run this task on
     * @param shouldOverrideConflictingTasks Whether this task should override conflicting tasks
     */
    public RunForTask(double timeout, Runnable callback, BunyipsSubsystem dependency, boolean shouldOverrideConflictingTasks) {
        super(timeout, dependency, shouldOverrideConflictingTasks);
        this.callback = callback;
    }

    @Override
    public void init() {
        // no-op
    }

    @Override
    public void periodic() {
        callback.run();
    }

    @Override
    public void onFinish() {
        // no-op
    }

    @Override
    public boolean isTaskFinished() {
        // Timeout will handle this automatically
        return false;
    }
}
