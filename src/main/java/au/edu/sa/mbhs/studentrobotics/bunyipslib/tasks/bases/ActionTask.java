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
import java.util.stream.Collectors;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.TaskGroup;

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
public class ActionTask extends TaskGroup {
    private final Action parentAction;
    private boolean actionFinished = false;

    /**
     * Wrap a new {@link Action} to be a {@link Task}.
     *
     * @param action The action to wrap
     */
    public ActionTask(@NonNull Action action) {
        parentAction = action;
        allActions(a -> setTasks(a.stream()
                .filter(t -> t instanceof Task)
                .map(t -> (Task) t)
                .collect(Collectors.toList())
        ));
        withName("Action :: " + action.getClass().getSimpleName());
        currentAction(ac -> {
            if (ac instanceof Task task) {
                withName(task.toString());
                setTimeout(task.getTimeout());
            }
        });
        allActions(actions -> {
            if (action instanceof SequentialAction) {
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
            if (action instanceof ParallelAction) {
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

    private void allActions(Consumer<List<Action>> t) {
        List<Action> actions = new ArrayList<>();
        if (parentAction instanceof SequentialAction seq) {
            actions.addAll(seq.getInitialActions());
        } else if (parentAction instanceof ParallelAction par) {
            actions.addAll(par.getInitialActions());
        } else {
            actions.add(parentAction);
        }
        t.accept(actions);
    }

    @SuppressWarnings("unchecked")
    private void currentAction(Consumer<Action> t) {
        // We can unwrap a SequentialAction to get the currently running contained task, if possible
        if (parentAction instanceof SequentialAction seq) {
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
                t.accept(ac);
            }
        } else {
            // We can't unwrap ParallelAction dynamically, so we'll just ignore it.
            // This is because we can't have one definitive task to track the status of, since they're all running in parallel
            t.accept(parentAction);
        }
    }

    @Override
    protected void periodic() {
        parentAction.preview(fieldOverlay);
        // TODO: reconsider the entire task execution system for subsystems - we really shouldn't rely on the group to do this for us
        //         we should just be able to run the task and it will internally schedule it!
        actionFinished = !parentAction.run(p);
        currentAction(ac -> {
            if (ac instanceof Task task)
                withName(task.toString());
        });
    }

    @Override
    protected boolean isTaskFinished() {
        return actionFinished;
    }

    @Override
    protected void onReset() {
        actionFinished = false;
        allActions(actions -> {
            for (Action action : actions) {
                if (action instanceof Task task) {
                    task.reset();
                }
            }
        });
    }
}
