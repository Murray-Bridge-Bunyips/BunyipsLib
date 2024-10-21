package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.OnceTask;

/**
 * A task to run a callback before immediately completing.
 * <p>
 * {@code new RunTask(() -> telemetry.add("Hello world"));}
 *
 * @since 1.0.0-pre
 */
public class RunTask extends OnceTask {
    private final Runnable callback;

    /**
     * Run the given callback immediately.
     *
     * @param callback The callback to run
     */
    public RunTask(@NonNull Runnable callback) {
        this.callback = callback;
        withName("Run");
    }

    /**
     * Run nothing and complete, this is useful as a placeholder task.
     */
    public RunTask() {
        this(() -> {
        });
        withName("Run Nothing");
    }

    @Override
    protected void runOnce() {
        callback.run();
    }
}
