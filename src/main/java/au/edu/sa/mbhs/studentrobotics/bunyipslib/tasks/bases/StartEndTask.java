package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

/**
 * Define a callback to run when this task is started, and when it is finished.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class StartEndTask extends Task {
    private final Runnable onStart;
    private final Runnable onFinish;

    /**
     * Create a new StartEndTask.
     *
     * @param onStart  the callback to run when the task starts
     * @param onFinish the callback to run when the task finishes
     */
    public StartEndTask(@NonNull Runnable onStart, @NonNull Runnable onFinish) {
        this.onStart = onStart;
        this.onFinish = onFinish;
        withName("Start End");
    }

    @Override
    protected void init() {
        onStart.run();
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected void onFinish() {
        onFinish.run();
    }
}
