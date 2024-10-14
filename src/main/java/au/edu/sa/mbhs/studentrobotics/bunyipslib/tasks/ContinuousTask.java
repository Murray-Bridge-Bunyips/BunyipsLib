package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ForeverTask;

/**
 * A task to run continuously and will never finish.
 * <p>
 * {@code new ContinuousTask(() -> telemetry.add("I will run forever"));}
 *
 * @since 1.0.0-pre
 */
public class ContinuousTask extends ForeverTask {
    private final Runnable callback;

    /**
     * A simple callback to run forever.
     *
     * @param callback the task to run continuously
     */
    public ContinuousTask(Runnable callback) {
        this.callback = callback;
        withName("Continuous");
    }

    @Override
    protected void periodic() {
        callback.run();
    }
}
