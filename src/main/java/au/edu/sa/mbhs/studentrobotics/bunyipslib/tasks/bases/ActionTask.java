package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;

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
        ifSingleTask(task -> {
            withName(task.toString());
            setTimeout(task.getTimeout());
        });
        ifManyActions(actions -> {
            if (action instanceof SequentialAction seq) {
                // For sequential actions, try to set a timeout based on the sum of all the inner task timeouts,
                // unless they are not tasks then we just use an infinite timeout
                if (actions.stream().anyMatch(a -> !(a instanceof Task) || ((Task) a).getTimeout().lte(INFINITE_TIMEOUT))) {
                    setTimeout(INFINITE_TIMEOUT);
                    return;
                }
                setTimeout(
                        actions.stream()
                                .map(a -> ((Task) a).getTimeout())
                                .reduce(Seconds.zero(), Measure::plus)
                );
            }
            if (action instanceof ParallelAction par) {
                // Parallel actions we can just set the timeout to the max of all the inner task timeouts, similar to above
                if (actions.stream().anyMatch(a -> !(a instanceof Task) || ((Task) a).getTimeout().lte(INFINITE_TIMEOUT))) {
                    setTimeout(INFINITE_TIMEOUT);
                    return;
                }
                setTimeout(
                        actions.stream()
                                .map(a -> ((Task) a).getTimeout())
                                .reduce(Seconds.zero(), Measure::max)
                );
            }
            // Set the name to the combination of all the inner actions
            StringBuilder name = new StringBuilder();
            for (int i = 0; i < actions.size(); i++) {
                name.append(actions.get(i));
                if (i != actions.size() - 1)
                    name.append(",");
            }
            withName(name.toString());
        });
    }

    private void ifManyActions(Consumer<List<Action>> t) {
        List<Action> actions = new ArrayList<>();
        if (action instanceof SequentialAction seq) {
            actions.addAll(seq.getInitialActions());
        } else if (action instanceof ParallelAction par) {
            actions.addAll(par.getInitialActions());
        } else {
            actions.add(action);
        }
        t.accept(actions);
    }

    @SuppressWarnings("unchecked")
    private void ifSingleTask(Consumer<Task> t) {
        if (action instanceof Task task) {
            t.accept(task);
            return;
        }
        // We can unwrap a SequentialAction to get the currently running contained task, if possible
        if (action instanceof SequentialAction seq) {
            List<Action> actions;
            try {
                Field actionsField = SequentialAction.class.getDeclaredField("actions");
                actionsField.setAccessible(true);
                actions = (List<Action>) actionsField.get(seq);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new RuntimeException("Failed to access an internal field, this shouldn't happen!", e);
            }
            assert actions != null;
            if (actions.isEmpty()) return;
            Action ac = actions.get(0);
            if (ac instanceof Task) {
                t.accept((Task) ac);
            }
        }
        // We can't unwrap ParallelAction dynamically, so we'll just ignore it.
        // This is because we can't have one definitive task to track the status of, since they're all running in parallel
    }

    @Override
    protected void periodic() {
        action.preview(fieldOverlay);
        actionFinished = !action.run(p);
        ifSingleTask(task -> withName(task.toString()));
    }

    @Override
    protected boolean isTaskFinished() {
        return actionFinished;
    }

    @Override
    protected void onFinish() {
        ifManyActions(actions -> {
            for (Action action : actions) {
                if (action instanceof Task task) {
                    task.finishNow();
                }
            }
        });
    }

    @Override
    protected void onReset() {
        actionFinished = false;
        ifManyActions(actions -> {
            for (Action action : actions) {
                if (action instanceof Task task) {
                    task.reset();
                }
            }
        });
    }
}
