package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A task to run continuously and will never finish on its own.
 * <p>
 * {@code new ContinuousTask(() -> telemetry.add("I will run forever until something interrupts me"));}
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class ContinuousTask extends Task {
    private final Runnable callback;

    /**
     * A simple callback to run continuously.
     *
     * @param callback the task to run continuously
     */
    public ContinuousTask(@NonNull Runnable callback) {
        this.callback = callback;
        withName("Continuous");
    }

    @Override
    protected void periodic() {
        callback.run();
    }

    @Override
    protected boolean isTaskFinished() {
        return false;
    }
}
