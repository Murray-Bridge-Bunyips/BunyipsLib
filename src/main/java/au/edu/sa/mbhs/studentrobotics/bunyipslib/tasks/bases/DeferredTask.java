package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

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
    private static final String SUFFIX = " (dfr.)";
    private final Supplier<Task> lazyTask;
    private Task builtTask;

    /**
     * Construct a new DeferredTask to run.
     *
     * @param lazyTask the task to construct and run when the DeferredTask starts running.
     */
    public DeferredTask(@NonNull Supplier<Task> lazyTask) {
        this.lazyTask = lazyTask;
        super.named("Task" + SUFFIX);
        disableSubsystemAttachment = true;
    }

    /**
     * @return the built task following deferral initialisation, nullable if this task hasn't been built yet
     */
    @Nullable
    public Task getTask() {
        return builtTask;
    }

    @Override
    protected void init() {
        builtTask = lazyTask.get();
        String name = builtTask.toString();
        Measure<Time> t = builtTask.timeout;
        Dbg.logd(getClass(), "built -> % (t=%)", name, t.magnitude() <= 0 ? "inf" : t.in(Seconds) + "s");
        if (("Task" + SUFFIX).equals(toString()))
            super.named(name);
        if (timeout.equals(INFINITE_TIMEOUT))
            timeout = t;
    }

    @Override
    protected void periodic() {
        if (builtTask == null) return;
        builtTask.isPriority = isPriority;
        builtTask.execute();
    }

    @Override
    protected void onFinish() {
        if (builtTask == null) return;
        builtTask.finish();
    }

    @Override
    protected void onReset() {
        if (builtTask == null) return;
        builtTask.reset();
        super.named("Task" + SUFFIX);
        timeout(INFINITE_TIMEOUT);
        builtTask = null;
    }

    @Override
    protected boolean isTaskFinished() {
        if (builtTask == null) return false;
        return builtTask.isFinished();
    }

    /**
     * Set the name of this DeferredTask. Note that " (dfr)" will be appended to indicate this task is deferred.
     * If the task is constructed, this method will no-op. Use the wrapped Task to set a name.
     */
    @NonNull
    @Override
    public final Task named(@Nullable String name) {
        if (builtTask != null)
            return this;
        super.named(name + SUFFIX);
        return this;
    }
}
