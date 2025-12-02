package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.ParallelTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.RaceTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.SequentialTaskGroup;

/**
 * This task is used as a wrapper for converting base RoadRunner {@link Action} implementations
 * into conventional {@link Task} instances. This task will also internally attempt to run the action in the context
 * of a {@link DualTelemetry} packet if running in a {@link BunyipsOpMode}, for seamless integration.
 * <p>
 * Do note that the {@link Task} implements {@link Action}, so if it is possible, directly converting the action
 * into a task at its definition is more efficient.
 * <p>
 * Tasks built with RoadRunner drives, despite being represented with Actions are internally composed of {@link Task} instances,
 * so {@link SequentialAction} and {@link ParallelAction} actions will also try to be unwrapped, with their full name available at init.
 * Actions that are Tasks will try to be run as closely as possible to their original {@link Task} implementation.
 * <p>
 * Do note that this task is a task wrapper, and will discard already assigned timeout, name, priority, and subsystem information
 * to match the parent action. Changes to these properties must be done to the child task, which will be reflected upwards.
 * Updating this wrapper will result in resets.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class ActionTask extends Task {
    /**
     * The underlying {@link Action} wrapped by this task. May exist as a {@link Task} instance.
     */
    public final Action parentAction;
    private boolean actionFinished = false;

    /**
     * Wrap a new {@link Action} to be a {@link Task}.
     *
     * @param action The action to wrap
     */
    public ActionTask(@NonNull Action action) {
        // Recursively unwrap the action if it can be converted into a TaskGroup
        if (action instanceof ParallelAction par)
            parentAction = new ParallelTaskGroup(par.getInitialActions().stream().map(ActionTask::new).toArray(ActionTask[]::new));
        else if (action instanceof SequentialAction seq)
            parentAction = new SequentialTaskGroup(seq.getInitialActions().stream().map(ActionTask::new).toArray(ActionTask[]::new));
        else if (action instanceof RaceAction race)
            parentAction = new RaceTaskGroup(race.getActions().stream().map(ActionTask::new).toArray(ActionTask[]::new));
        else if (action instanceof SleepAction sleep)
            parentAction = new WaitTask(sleep.getDt(), Seconds);
        else if (action instanceof InstantAction ins)
            parentAction = new Lambda(ins.getF()::run);
        else if (action instanceof NullAction)
            parentAction = new Lambda();
        else
            parentAction = action;
        named(parentAction instanceof Task ? parentAction.toString() : parentAction.getClass().getSimpleName());
        if (parentAction instanceof Task task)
            sync(task);
    }

    @Override
    protected void periodic() {
        parentAction.preview(dashboard.fieldOverlay());
        actionFinished = !parentAction.run(dashboard);
        if (parentAction instanceof Task task)
            sync(task);
    }

    @Override
    protected boolean isTaskFinished() {
        return actionFinished;
    }

    @Override
    protected void onFinish() {
        if (parentAction instanceof Task task)
            task.finish();
    }

    @Override
    protected void onReset() {
        actionFinished = false;
        if (parentAction instanceof Task task)
            task.reset();
    }
}
