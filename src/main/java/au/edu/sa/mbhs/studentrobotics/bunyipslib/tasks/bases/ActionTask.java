package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;

/**
 * This base task is used as a wrapper for converting base RoadRunner {@link Action} implementations
 * into conventional {@link Task} instances. This task will also internally attempt to run the action in the context
 * of a {@link DualTelemetry} packet if running in a {@link BunyipsOpMode}, for seamless integration.
 * <p>
 * Do note that the {@link Task} implements {@link Action}, so if it is possible, directly converting the action
 * into a task at it's definition is more efficient.
 * <p>
 * Tasks built with RoadRunner, despite being represented with Actions are internally composed of {@link Task} instances.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class ActionTask extends Task {
    private final Action action;
    private boolean actionFinished = false;

    /**
     * Wrap a new {@link Action} to be a {@link Task}.
     *
     * @param action The action to wrap
     */
    public ActionTask(@NonNull Action action) {
        this.action = action;
        withName("Action :: " + action.getClass().getSimpleName());
    }

    @Override
    protected void periodic() {
        action.preview(fieldOverlay);
        actionFinished = !action.run(p);
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
