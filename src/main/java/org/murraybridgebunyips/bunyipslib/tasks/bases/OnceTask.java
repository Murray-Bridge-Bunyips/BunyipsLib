package org.murraybridgebunyips.bunyipslib.tasks.bases;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs once and then immediately completes.
 */
public abstract class OnceTask extends Task {
    protected OnceTask() {
        super(Seconds.zero());
    }

    protected OnceTask(BunyipsSubsystem dependencySubsystem, boolean override) {
        super(Seconds.zero(), dependencySubsystem, override);
    }

    @Override
    protected final void init() {
        runOnce();
    }

    @Override
    protected final void periodic() {
    }

    @Override
    protected final boolean isTaskFinished() {
        // OnceTasks may sometimes have their timeouts adjusted at runtime
        return getTimeout().magnitude() == 0.0;
    }

    @Override
    protected final void onFinish() {
    }

    /**
     * Code to run once.
     */
    protected abstract void runOnce();
}
