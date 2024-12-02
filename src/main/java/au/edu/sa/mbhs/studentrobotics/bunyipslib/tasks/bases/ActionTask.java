package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;

/**
 * This task is used as a wrapper for converting base RoadRunner {@link Action} implementations
 * into conventional {@link Task} instances. This task will also internally attempt to run the action in the context
 * of a {@link DualTelemetry} packet if running in a {@link BunyipsOpMode}, for seamless integration.
 * <p>
 * Do note that the {@link Task} implements {@link Action}, so if it is possible, directly converting the action
 * into a task at it's definition is more efficient.
 * <p>
 * Tasks built with RoadRunner drives, despite being represented with Actions are internally composed of {@link Task} instances,
 * so {@link SequentialAction} and {@link ParallelAction} actions will also try to be unwrapped, with their full name available at init.
 * Actions that are Tasks will try to be run as closely as possible to their original {@link Task} implementation.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class ActionTask extends Task {
    private Action parentAction;
    private boolean actionFinished = false;

    /**
     * Wrap a new {@link Action} to be a {@link Task}.
     *
     * @param action The action to wrap
     */
    public ActionTask(@NonNull Action action) {
        parentAction = action;// TODO: roadrunner actions aren't working and they just continue their powers
        // Recursively unwrap the action if it can be converted into a TaskGroup
//        if (parentAction instanceof ParallelAction par) {
//            parentAction = new ParallelTaskGroup(par.getInitialActions().stream().map(ActionTask::new).toArray(ActionTask[]::new));
//        }
//        if (parentAction instanceof SequentialAction seq) {
//            parentAction = new SequentialTaskGroup(seq.getInitialActions().stream().map(ActionTask::new).toArray(ActionTask[]::new));
//        }
//        withName(parentAction instanceof Task ? parentAction.toString() : parentAction.getClass().getSimpleName());
//        if (parentAction instanceof Task task) {
//            withTimeout(task.getTimeout());
//            if (task.getDependency().isPresent())
//                onSubsystem(task.getDependency().get());
//        }
    }

    @Override
    protected void periodic() {
        parentAction.preview(fieldOverlay);
        actionFinished = !parentAction.run(p);
        if (parentAction instanceof Task task) {
            withTimeout(task.getTimeout());
//            withName(task.toString()); // TODO: respect names better
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return actionFinished;
    }

    @Override
    protected void onReset() {
        actionFinished = false;
    }
}
