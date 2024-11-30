package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Exceptions;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Represents a task that is constructed at runtime. This is useful for tasks that have runtime requirements
 * that cannot be determined when this task is made, such as RoadRunner tasks that run on a non-structural basis.
 * <p>
 * Note some caveats of using this task is that timeout and other information related to this task is unknown until
 * this task is run once, in which this data will be updated to match the task that was built. If this task
 * is to run on a subsystem, it <b>must be declared on the inner task</b>, as DeferredTask does not have enough information
 * to know where to run, since these allocations are done at construction.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class DeferredTask extends Task {
    private static final String UNCONSTRUCTED_NAME = " (dfr)";
    private final Supplier<Task> lazyTask;
    private Task builtTask;

    /**
     * Construct a new DeferredTask to run.
     *
     * @param lazyTask the task to construct and run when the DeferredTask starts running.
     */
    public DeferredTask(@NonNull Supplier<Task> lazyTask) {
        this.lazyTask = lazyTask;
        // We're not actually a task, so we'll let the inner task manage reports
        muteReports();
        super.withName(UNCONSTRUCTED_NAME);
    }

    @Override
    protected void init() {
        builtTask = lazyTask.get();
        String name = builtTask.toString();
        Measure<Time> timeout = builtTask.getTimeout();
        Dbg.logd(getClass(), "built -> % (t=%)", name, timeout.magnitude() <= 0 ? "inf" : timeout.in(Seconds) + "s");
        if (UNCONSTRUCTED_NAME.equals(toString()))
            super.withName(name);
        if (getTimeout().equals(INFINITE_TIMEOUT))
            setTimeout(timeout);
        builtTask.getDependency().ifPresent((dep) ->
                dep.setHighPriorityCurrentTask(builtTask));
    }

    @Override
    protected void periodic() {
        if (builtTask == null) return;
        if (!builtTask.hasDependency())
            builtTask.run();
    }

    @Override
    protected void onFinish() {
        if (builtTask == null) return;
        builtTask.finishNow();
        builtTask.getDependency().ifPresent(BunyipsSubsystem::cancelCurrentTask);
    }

    @Override
    protected void onReset() {
        if (builtTask == null) return;
        builtTask.reset();
        withName(UNCONSTRUCTED_NAME);
        withTimeout(INFINITE_TIMEOUT);
        builtTask = null;
    }

    @Override
    protected boolean isTaskFinished() {
        if (builtTask == null) return false;
        return builtTask.hasDependency() ? builtTask.isFinished() : builtTask.pollFinished();
    }

    /**
     * Set the name of this DeferredTask. Note that "(dyn)" will be appended to indicate this task is not constructed.
     * If the task is constructed, this method will no-op. Use the wrapped Task to set a name.
     */
    @NonNull
    @Override
    public final Task withName(@Nullable String name) {
        if (builtTask != null)
            return this;
        super.withName(name + " (dyn)");
        return this;
    }

    /**
     * Subsystems cannot be attached to DynamicTasks. This method will no-op and log an error.
     */
    @NonNull
    @Override
    public final Task onSubsystem(@NonNull BunyipsSubsystem subsystem, boolean override) {
        StackTraceElement f = Exceptions.getCallingUserCodeFunction();
        Dbg.error(f, "Dynamic tasks are not designed to be attached to a subsystem, as the internal task will be scheduled to subsystems instead.");
        opMode(o -> o.telemetry.log(f, Text.html().color("red", "error: ").text("dynamic tasks should not be attached to subsystems!")));
        return this;
    }
}
