package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

/**
 * A task that runs a given task and, if a condition is met,
 * retries it (or a different task) up to a specified number of times.
 * <p>
 * This task is useful for actions that may not succeed on the first attempt
 * and require re-running, such as vision alignment or precise mechanism movement.
 * <p>
 * This task is implemented from the Marrow library by FTC 23644, Skeleton Army.
 * <a href="https://github.com/Skeleton-Army/Marrow/blob/1fa47b6e286f5925c4e44e9ce1afeefbd9c22040/customLibraries/solverslib/src/main/java/com/skeletonarmy/marrow/solverslib/RetryCommand.java">Source</a>
 *
 * @since 7.5.0
 */
public class RetryTask extends Task {
    private final Task task;
    private final Task retryTask;
    private final BooleanSupplier shouldRetry;
    private final int maxRetries;

    private Task currentTask;
    private int retryCount = 0;
    private boolean isFinished = false;

    /**
     * Creates a new RetryTask.
     *
     * @param task        Supplies the task to run on the first attempt.
     * @param retryTask   The task to run on retries if the retry condition fails.
     * @param shouldRetry A condition that returns {@code true} if a retry should be attempted, or {@code false} if the task should finish without retrying.
     * @param maxRetries  The maximum number of retries allowed.
     */
    public RetryTask(@NonNull Task task, @NonNull Task retryTask, BooleanSupplier shouldRetry, int maxRetries) {
        disableSubsystemAttachment = true;
        this.task = task;
        this.retryTask = retryTask;
        this.shouldRetry = shouldRetry;
        this.maxRetries = maxRetries;
        named(task + " || " + retryTask + " (rty.)");
        timeout = task.timeout.times(Math.max(1, maxRetries));
    }

    /**
     * Creates a new RetryTask where the retry task is the same as the initial one.
     *
     * @param task        A supplier that creates a new instance of the task to run.
     * @param shouldRetry A condition that returns {@code true} if a retry should be attempted, or {@code false} if the task should finish without retrying.
     * @param maxRetries  The maximum number of retries allowed.
     */
    public RetryTask(@NonNull Task task, BooleanSupplier shouldRetry, int maxRetries) {
        this(task, task, shouldRetry, maxRetries);
    }

    @Override
    protected void init() {
        isFinished = false;
        retryCount = 0;
        currentTask = task;
    }

    @Override
    protected void periodic() {
        if (!currentTask.poll()) {
            named(currentTask.toString());
            timeout = currentTask.timeout.times(maxRetries - retryCount + 1);
            currentTask.execute();
            return;
        }

        currentTask.finishNow();

        // Check if we should retry
        if (retryCount < maxRetries && shouldRetry.getAsBoolean()) {
            retryCount++;
            currentTask = retryTask;
            currentTask.reset();
        } else {
            isFinished = true;
        }
    }

    @Override
    protected void onFinish() {
        currentTask.finishNow();
    }

    @Override
    protected void onReset() {
        isFinished = false;
        retryCount = 0;
        currentTask.reset();
    }

    @Override
    protected boolean isTaskFinished() {
        return isFinished;
    }
}
