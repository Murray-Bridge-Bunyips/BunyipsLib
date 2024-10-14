package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A task to run for a timeout.
 * <p>
 * {@code new RunForTask(5, arm::spin, () -> { // optional finish callback });}
 *
 * @since 1.0.0-pre
 */
public class RunForTask extends Task {
    private final Runnable callback;
    private Runnable finishCallback;

    /**
     * A runnable that will run until timeout.
     *
     * @param timeout  The time to run the task for
     * @param callback The callback to run every loop
     */
    public RunForTask(Measure<Time> timeout, Runnable callback) {
        super(timeout);
        this.callback = callback;
        withName("Run For");
    }

    /**
     * A runnable that will run until timeout, with a finish callback.
     *
     * @param timeout        The time to run the task for
     * @param callback       The callback to run every loop
     * @param finishCallback The callback to run after this task finishes
     */
    public RunForTask(Measure<Time> timeout, Runnable callback, Runnable finishCallback) {
        super(timeout);
        this.callback = callback;
        this.finishCallback = finishCallback;
        withName("Run For");
    }

    @Override
    protected void periodic() {
        callback.run();
    }

    @Override
    protected void onFinish() {
        if (finishCallback != null)
            finishCallback.run();
    }

    @Override
    protected boolean isTaskFinished() {
        // Timeout will handle this automatically
        return false;
    }
}
