package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.Nullable;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.Supplier;

/**
 * Represents a task that is constructed at runtime. This is useful for tasks that have runtime requirements
 * that cannot be determined when this task is made, such as RoadRunner tasks that run on a non-structural basis.
 * <p>
 * Note some caveats of using this task is that timeout and other information related to this task is unknown until
 * this task is run once, in which this data will be updated to match the task that was built. If this task
 * is to run on a subsystem, it <b>must be declared on the inner task</b>, as DynamicTask does not have enough information
 * to know where to run, since these allocations are done at construction.
 *
 * @author Lucas Bubner, 2024
 */
public class DynamicTask extends NoTimeoutTask {
    private final Supplier<Task> lazyTask;

    @Nullable
    private Task builtTask;

    /**
     * Construct a new DynamicTask to run.
     *
     * @param lazyTask the task to construct and run when the DynamicTask starts running.
     */
    public DynamicTask(Supplier<Task> lazyTask) {
        this.lazyTask = lazyTask;
        // We're not actually a task, so we'll let the inner task manage reports
        withMutedReports();
        withName("Dynamic (Pending construction)");
    }

    @Override
    protected void init() {
        builtTask = lazyTask.get();
        String name = builtTask.toString();
        Measure<Time> timeout = builtTask.getTimeout();
        Dbg.logd(getClass(), "built -> % (t=%)", name, timeout.magnitude() <= 0 ? "inf" : timeout.in(Seconds) + "s");
        withName(name);
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
        withName("Dynamic (Pending construction)");
        withTimeout(INFINITE_TIMEOUT);
        builtTask = null;
    }

    @Override
    protected boolean isTaskFinished() {
        if (builtTask == null) return false;
        return builtTask.hasDependency() ? builtTask.isFinished() : builtTask.pollFinished();
    }
}