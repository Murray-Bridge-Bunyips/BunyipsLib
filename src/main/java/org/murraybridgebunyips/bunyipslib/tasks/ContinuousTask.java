package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * A task to run continuously and will never finish.
 * <p>
 * {@code new ContinuousTask(() -> addTelemetry("I will run forever"));}
 */
public class ContinuousTask extends ForeverTask {
    private final Runnable callback;

    /**
     * Subsystem-independent continuous task.
     *
     * @param callback the task to run continuously
     */
    public ContinuousTask(Runnable callback) {
        this.callback = callback;
    }

    /**
     * Subsystem-dependent continuous task.
     *
     * @param callback                       the task to run continuously
     * @param dependency                     the subsystem to depend on
     * @param shouldOverrideConflictingTasks whether to override conflicting tasks on this subsystem
     */
    public ContinuousTask(Runnable callback, BunyipsSubsystem dependency, boolean shouldOverrideConflictingTasks) {
        super(dependency, shouldOverrideConflictingTasks);
        this.callback = callback;
    }

    @Override
    protected void periodic() {
        callback.run();
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
