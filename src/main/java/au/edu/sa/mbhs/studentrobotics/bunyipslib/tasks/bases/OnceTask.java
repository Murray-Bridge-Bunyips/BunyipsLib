package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

/**
 * A short task that runs once and then immediately completes.
 *
 * @since 1.0.0-pre
 */
public abstract class OnceTask extends Task {
    // This value may need adjustment in combination with a SequentialTaskGroup where the timeouts are summed,
    // however we can't tell how long a single method will execute for so we need to assume OnceTasks will only
    // do one small task.
    /**
     * The epsilon value for the OnceTask timeout.
     */
    public static final int EPSILON_MS = 10;

    protected OnceTask() {
        // For OnceTasks, we can't have an infinite timeout but we can use a very very short one instead
        // This is so the schedulers do not mistake this task as for one that will end up running forever,
        // as all OnceTasks will run only once. This also helps telemetry decide how long a task will execute for.
        super(Milliseconds.of(EPSILON_MS));
    }

    @Override
    protected final void init() {
        runOnce();
    }

    @Override
    protected final void periodic() {
        // no-op
    }

    @Override
    protected final boolean isTaskFinished() {
        // OnceTasks may sometimes have their timeouts adjusted at runtime
        return getTimeout().lte(Milliseconds.of(EPSILON_MS));
    }

    @Override
    protected final void onFinish() {
        // no-op
    }

    /**
     * Code to run once.
     */
    protected abstract void runOnce();
}
