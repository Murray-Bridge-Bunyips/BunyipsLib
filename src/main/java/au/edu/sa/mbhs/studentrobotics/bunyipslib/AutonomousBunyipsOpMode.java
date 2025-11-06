package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.executables.UserSelection;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ActionTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.DeferredTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.SequentialTaskGroup;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Ref;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;
import dev.frozenmilk.util.cell.RefCell;

/**
 * {@link BunyipsOpMode} variant for Autonomous operation. Uses the {@link Task} system for a queued action OpMode.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 * @see BunyipsOpMode
 * @since 1.0.0-pre
 */
public abstract class AutonomousBunyipsOpMode extends BunyipsOpMode {
    /**
     * Used for tasks that have no timeout to generate an "estimate to OpMode completion" metric.
     * Purely visual, and does not affect the actual task (hence why this field is not exposed to FtcDashboard).
     */
    public static double INFINITE_TASK_ASSUMED_DURATION_SECONDS = 5.0;
    private final ConcurrentLinkedDeque<Task> tasks = new ConcurrentLinkedDeque<>();
    // Pre- and post-queues cannot have their tasks removed, so we can rely on their .size() methods
    private final ConcurrentLinkedDeque<Task> postQueue = new ConcurrentLinkedDeque<>();
    private final ConcurrentLinkedDeque<Task> preQueue = new ConcurrentLinkedDeque<>();
    private Object[] selections = {};
    @NonNull
    private HashSet<BunyipsSubsystem> updatedSubsystems = new HashSet<>();
    private int taskCount;
    private UserSelection<?> userSelection;
    private int currentTask = 1;
    private volatile boolean safeToAddTasks;
    private volatile boolean callbackReceived;
    private OnTasksDone onTasksDone = OnTasksDone.FINISH_OPMODE;
    private boolean tasksFinished;

    private void callback(@Nullable Object selectedOpMode) {
        // Safety as the OpMode may not be running anymore (due to it being on another thread)
        if (isStopRequested())
            return;

        safeToAddTasks = true;

        Exceptions.runUserMethod(() -> onReady(selectedOpMode == null ? null : Ref.of(selectedOpMode)));
        callbackReceived = true;

        // Add any queued tasks that were delayed previously and we can do now
        synchronized (postQueue) {
            for (Task task : postQueue) {
                add(task);
            }
            postQueue.clear();
        }
        synchronized (preQueue) {
            for (Task task : preQueue) {
                addFirst(task);
            }
            preQueue.clear();
        }

        String timeLeft = getApproximateTimeLeft();
        Text.Builder out = Text.builder();
        out.append("[AutonomousBunyipsOpMode] onReady() called | %% task(s) queued% | % subsystem(s)\n",
                userSelection != null ? "usr: " + Text.removeHtml(Ref.stringify(selectedOpMode)) + " | " : "",
                taskCount,
                timeLeft.isEmpty() ? "" : timeLeft + " to complete",
                updatedSubsystems.size()
        );
        for (Task task : tasks) {
            out.append("   -> %\n", task.toVerboseString());
        }
        out.append("\n");
        for (BunyipsSubsystem subsystem : updatedSubsystems) {
            out.append("   :: %\n", subsystem.toVerboseString());
        }
        Dbg.logd(out.toString());

        resume();
    }

    @Override
    protected final void onInit() {
        // Run user-defined hardware initialisation
        Exceptions.runUserMethod(this::onInitialise);
        if (updatedSubsystems.isEmpty()) {
            // We might be using an implicit subsystem initialisation schema, look for static instances instead
            updatedSubsystems = BunyipsSubsystem.getInstances();
        }
        if (updatedSubsystems.isEmpty()) {
            Dbg.warn(getClass(), "Caution: No subsystems are attached to AutonomousBunyipsOpMode that can be updated.");
        }
        if (selections.length > 1) {
            // There is minimum two options so we proceed with selection
            Threads.start("abom user selection", userSelection);
        } else {
            // There are no selections to make, so just run the callback with whatever we have, or if the user did
            // not mention any selections we give them an empty cell
            callback(selections.length == 1 ? selections[0] : Ref.empty());
        }
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike {@link #onInitDone}, this will only execute once play is hit and not when initialisation is done.
     * <p>
     * If overriding this method, it is strongly recommended to call {@code super.onStart()} in your method to
     * ensure that the asynchronous task allocation has been notified to stop immediately. This is
     * not required if {@link #setOpModes(Object...)} returns null.
     */
    @Override
    protected void onStart() {
        if (isStopRequested())
            return;
        if (userSelection != null) {
            // UserSelection will internally check opMode.isInInit() to see if it should terminate itself,
            // but we should cancel here too to be pedantic
            Threads.stop(userSelection);
        }
        // Awaiting thread joining
        if (!callbackReceived)
            halt();
    }

    @Override
    protected final void activeLoop() {
        // Run any code defined by the user
        Exceptions.runUserMethod(this::periodic);

        // Update all subsystems which may also contain user routines
        // This also ensures the subsystems are ready to accept incoming tasks
        for (BunyipsSubsystem subsystem : updatedSubsystems) {
            subsystem.update();
        }

        if (tasksFinished && onTasksDone == OnTasksDone.CONTINUE_EXECUTION) {
            // Tasks are all done and all we need to do is update subsystems, so we exit here
            return;
        }

        // Run the queue of tasks
        synchronized (tasks) {
            Task currentTask = tasks.peekFirst();
            if (currentTask == null) {
                telemetry.log("<font color='gray'>auto:</font> tasks done -> %", onTasksDone);
                if (onTasksDone != OnTasksDone.CONTINUE_EXECUTION) {
                    finish(onTasksDone == OnTasksDone.FINISH_OPMODE);
                }
                telemetry.overheadSubtitle = Text.format("<small><font color='gray'>All tasks completed in <b>%s</b>.</font></small>", Mathf.round(timer.elapsedTime().in(Seconds), 2));
                tasksFinished = true;
                return;
            }

            telemetry.overheadSubtitle = Text.format("<small><font color='aqua'>Running task <b>%/%</b></font> | %%</small>",
                    this.currentTask, taskCount, currentTask, getApproximateTimeLeft());

            try {
                currentTask.execute();
                if (currentTask.isFinished()) {
                    tasks.removeFirst();
                    double runTime = currentTask.getElapsedTime().in(Seconds);
                    Dbg.logd("[AutonomousBunyipsOpMode] task %/% (%) finished%", this.currentTask, taskCount, currentTask, runTime != 0 ? " -> " + runTime + "s" : "");
                    this.currentTask++;
                }
            } catch (Exception e) {
                Exceptions.handle(e, telemetry::log);
            }
        }
    }

    /**
     * Use an init task if you wish to run looping code during the initialisation phase of the OpMode.
     *
     * @see #setInitTask
     */
    @Override
    protected final boolean onInitLoop() {
        return userSelection == null || !Threads.isRunning(userSelection);
    }

    /**
     * Sets the behaviour this OpMode should take when all tasks in the task queue have been completed.
     * Review the {@link OnTasksDone} enum for more information on the different modes.
     *
     * @param finishBehaviour the actions to take when all tasks are done, by default, {@link OnTasksDone#FINISH_OPMODE}.
     * @since 7.0.0
     */
    public final void setCompletionBehaviour(OnTasksDone finishBehaviour) {
        if (finishBehaviour == null) return;
        onTasksDone = finishBehaviour;
    }

    /**
     * Call to manually add the subsystems that should be managed by AutonomousBunyipsOpMode. Using this method will override
     * the automatic collection of {@link BunyipsSubsystem}s, and allows you to determine which subsystems will be managed for
     * this OpMode.
     * <p>
     * For most cases, using this method is not required and all you need to do is construct your subsystems, and they
     * will be managed automatically. This method is for advanced cases where you don't want this behaviour to happen.
     *
     * @param subsystems the restrictive list of subsystems to be managed and updated by ABOM
     */
    public final void use(@NonNull BunyipsSubsystem... subsystems) {
        if (Arrays.stream(subsystems).anyMatch(Objects::isNull)) {
            throw new RuntimeException("Null subsystems were added in the use() method!");
        }
        Collections.addAll(updatedSubsystems, subsystems);
    }

    /**
     * Call to add {@link Task} instances that will be executed sequentially during the active loop.
     *
     * @param newTask task to add to the run queue
     * @param <T>     the inherited task type
     * @return the added task
     */
    public final <T extends Task> T add(@NonNull T newTask) {
        checkTaskForDependency(newTask);
        if (!safeToAddTasks) {
            telemetry.log("<font color='gray'>auto:</font> <font color='yellow'>caution!</font> a task was added manually before the onReady callback");
        }
        synchronized (tasks) {
            insert(newTask);
        }
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added %/%", newTask, getTaskTimeout(newTask), taskCount, taskCount);
        return newTask;
    }

    // prereq: inside a synchronized block
    private void insert(Task task) {
        // Before we add the task we also want to unwrap a top-level SequentialTaskGroup which may be created through RR
        // This freshens up the sequential task numbering nature when using markers which are deconstructed
        // by the TaskBuilder, instead of scheduling a large 1/1 sequential task. Technically, this is just cosmetic.
        if (task instanceof SequentialTaskGroup seq) {
            // We only want to go one layer in, don't call recursively
            tasks.addAll(seq.tasks);
        } else if (task instanceof ActionTask ac && ac.parentAction instanceof Task innerTask) {
            // Try again with the wrapped task, common for RoadRunner builders
            insert(innerTask);
        } else {
            tasks.add(task);
        }
    }

    /**
     * Call to add {@link Task} instances that will be executed sequentially during the active loop.
     * This task will be internally wrapped into a {@link DeferredTask}, and added to the queue automatically.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param newDynamicTask deferred task to add to the run queue
     * @param <T>            the inherited task type
     * @return the added task
     */
    @SuppressWarnings("unchecked")
    public final <T extends Task> T defer(@NonNull Supplier<T> newDynamicTask) {
        return (T) add(new DeferredTask((Supplier<Task>) newDynamicTask));
    }

    /**
     * Implicitly constructs a new {@link Lambda} and adds it to the run queue.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda run(@NonNull Runnable runnable) {
        return add(new Lambda(runnable));
    }

    /**
     * Implicitly constructs a new {@link Lambda} and adds it to the run queue.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param name     the name of the task
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda run(@NonNull String name, @NonNull Runnable runnable) {
        Lambda task = new Lambda(runnable);
        task.named(name);
        return add(task);
    }

    /**
     * Implicitly constructs a new {@link WaitTask} and adds it to the run queue.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param duration the duration to wait
     * @return the added {@link WaitTask}
     */
    @NonNull
    public final WaitTask wait(@NonNull Measure<Time> duration) {
        return add(new WaitTask(duration));
    }

    /**
     * Implicitly constructs a new {@link WaitTask} and adds it to the run queue.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param duration the duration to wait
     * @param unit     the unit of the duration
     * @return the added {@link WaitTask}
     */
    @NonNull
    public final WaitTask wait(double duration, @NonNull Time unit) {
        return add(new WaitTask(unit.of(duration)));
    }

    /**
     * Insert a task at a specific index in the queue. This is useful for adding tasks that should be run
     * at a specific point in the autonomous sequence. Note that this function immediately produces side effects,
     * and subsequent calls will not be able to insert tasks at the same index due to the shifting of tasks.
     *
     * @param index   the index to insert the task at, starting from 0
     * @param newTask the task to add to the run queue
     * @param <T>     the inherited task type
     * @return the added task
     */
    public final <T extends Task> T addAtIndex(int index, @NonNull T newTask) {
        checkTaskForDependency(newTask);
        ArrayDeque<Task> tmp = new ArrayDeque<>();
        if (!safeToAddTasks) {
            telemetry.log("<font color='gray'>auto:</font> <font color='yellow'>caution!</font> a task was added manually before the onReady callback");
        }
        synchronized (tasks) {
            if (index < 0) {
                throw new IllegalArgumentException("Cannot insert task at index " + index + ", out of bounds");
            } else if (index > tasks.size()) {
                telemetry.log(getClass(), Text.html().color("red", "task index % is out of bounds. task was added to the end."), index);
                Dbg.error(getClass(), "Task index % out of bounds to insert task, appending task to end...", index);
                return addLast(newTask);
            }
            // Deconstruct the queue to insert the new task
            while (tasks.size() > index) {
                tmp.add(tasks.removeLast());
            }
            // Insert the new task
            insert(newTask);
            // Refill the queue
            while (!tmp.isEmpty()) {
                tasks.add(tmp.removeLast());
            }
        }
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> inserted %/%", newTask, getTaskTimeout(newTask), index, taskCount);
        return newTask;
    }

    /**
     * Insert a task at a specific index in the queue. This is useful for adding tasks that should be run
     * at a specific point in the autonomous sequence. Note that this function immediately produces side effects,
     * and subsequent calls will not be able to insert tasks at the same index due to the shifting of tasks.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
     * <p>
     * <b>Note:</b> You do not need to call {@link #addAtIndex(int, Task)} on this task, it is called for you automatically.
     *
     * @param index          the index to insert the task at, starting from 0
     * @param newDynamicTask deferred task to add to the run queue
     * @param <T>            the inherited task type
     * @return the added task
     */
    @SuppressWarnings("unchecked")
    public final <T extends Task> T deferAtIndex(int index, @NonNull Supplier<T> newDynamicTask) {
        return (T) addAtIndex(index, new DeferredTask((Supplier<Task>) newDynamicTask));
    }

    /**
     * Insert an implicit {@link Lambda} at a specific index in the queue.
     *
     * @param index    the index to insert the task at, starting from 0
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda addAtIndex(int index, @NonNull Runnable runnable) {
        return addAtIndex(index, new Lambda(runnable));
    }

    /**
     * Insert an implicit {@link Lambda} at a specific index in the queue.
     *
     * @param index    the index to insert the task at, starting from 0
     * @param name     the name of the task
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda addAtIndex(int index, @NonNull String name, @NonNull Runnable runnable) {
        Lambda task = new Lambda(runnable);
        task.named(name);
        return addAtIndex(index, task);
    }

    /**
     * Add a task to the run queue at a specified run queue priority.
     *
     * @param runQueuePriority the run queue priority.
     * @param newTask          task to add to the run queue
     * @param <T>              the inherited task type
     * @return the added task
     * @see #addLast(Task)
     * @see #addFirst(Task)
     */
    public final <T extends Task> T add(@NonNull TaskPriority runQueuePriority, @NonNull T newTask) {
        return switch (runQueuePriority) {
            case FIRST -> addFirst(newTask);
            case LAST -> addLast(newTask);
            default -> add(newTask);
        };
    }

    /**
     * Add a task to the run queue at a specified run queue priority.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param runQueuePriority the run queue priority.
     * @param newDynamicTask   deferred task to add to the run queue
     * @param <T>              the inherited task type
     * @return the added task
     * @see #addLast(Task)
     * @see #addFirst(Task)
     */
    @SuppressWarnings("unchecked")
    public final <T extends Task> T defer(@NonNull TaskPriority runQueuePriority, @NonNull Supplier<T> newDynamicTask) {
        return (T) add(runQueuePriority, new DeferredTask((Supplier<Task>) newDynamicTask));
    }

    /**
     * Add a task to the run queue, but after {@link #onReady(RefCell)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(RefCell)}.
     *
     * @param newTask task to add to the run queue
     * @param <T>     the inherited task type
     * @return the added task
     */
    public final <T extends Task> T addLast(@NonNull T newTask) {
        checkTaskForDependency(newTask);
        if (!callbackReceived) {
            synchronized (postQueue) {
                postQueue.add(newTask);
            }
            telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> queued end-init %/%", newTask, getTaskTimeout(newTask), postQueue.size(), postQueue.size());
            return newTask;
        }
        synchronized (tasks) {
            tasks.addLast(newTask);
        }
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added %/%", newTask, getTaskTimeout(newTask), taskCount, taskCount);
        return newTask;
    }

    /**
     * Add a task to the run queue, but after {@link #onReady(RefCell)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(RefCell)}.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param newDynamicTask deferred task to add to the run queue
     * @param <T>            the inherited task type
     * @return the added task
     */
    @SuppressWarnings("unchecked")
    public final <T extends Task> T deferLast(@NonNull Supplier<T> newDynamicTask) {
        return (T) addLast(new DeferredTask((Supplier<Task>) newDynamicTask));
    }

    /**
     * Add a task to the very start of the queue. This is useful to call when working with tasks that
     * should be queued at the very start of the autonomous, while still being able to add tasks
     * asynchronously with user input in {@link #onReady(RefCell)}.
     *
     * @param newTask task to add to the run queue
     * @param <T>     the inherited task type
     * @return the added task
     */
    public final <T extends Task> T addFirst(@NonNull T newTask) {
        checkTaskForDependency(newTask);
        if (!callbackReceived) {
            synchronized (preQueue) {
                preQueue.add(newTask);
            }
            telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> queued end-init 1/%", newTask, getTaskTimeout(newTask), preQueue.size());
            return newTask;
        }
        synchronized (tasks) {
            tasks.addFirst(newTask);
        }
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added 1/%", newTask, getTaskTimeout(newTask), taskCount);
        return newTask;
    }

    /**
     * Add a task to the very start of the queue. This is useful to call when working with tasks that
     * should be queued at the very start of the autonomous, while still being able to add tasks
     * asynchronously with user input in {@link #onReady(RefCell)}.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
     * <p>
     * <b>Note:</b> You do not need to call {@link #add(Task)} on this task, it is called for you automatically.
     *
     * @param newDynamicTask deferred task to add to the run queue
     * @param <T>            the inherited task type
     * @return the added task
     */
    @SuppressWarnings("unchecked")
    public final <T extends Task> T deferFirst(@NonNull Supplier<T> newDynamicTask) {
        return (T) addFirst(new DeferredTask((Supplier<Task>) newDynamicTask));
    }

    private String getTaskTimeout(Task task) {
        // Try to extract the timeout of the task, as Task does not have a timeout
        Measure<Time> timeout = task.timeout;
        // INFINITE_TASK is defined as Seconds.zero()
        return timeout.magnitude() != 0.0
                ? Mathf.round(timeout.in(Seconds), 1) + "s"
                : "∞";
    }

    private String getApproximateTimeLeft() {
        // Must use an atomic boolean due to lambda restrictions
        AtomicBoolean approx = new AtomicBoolean(false);
        // Attempt to get the time left for all tasks by summing their timeouts
        double timeLeft = tasks.stream().mapToDouble(task -> {
            // We cannot extract the duration of a task that is not a Task, we will return zero instead of the assumption
            // as they are completely out of our control, and we don't even know how they function
            Measure<Time> timeout = task.timeout;
            // We have to approximate and guess as we cannot determine the duration of a task that is infinite
            if (timeout.magnitude() == 0.0) {
                // We also adjust the approx flag so we can notify the user that the time is an estimate with a tilde
                approx.set(true);
                return INFINITE_TASK_ASSUMED_DURATION_SECONDS;
            }
            return timeout.in(Seconds);
        }).sum();
        // Determine the time left for all tasks
        Task curr = tasks.peekFirst();
        if (curr == null) return "";
        Measure<Time> timeout = curr.getElapsedTime();
        // Offset by the current task's time left to interpolate between tasks
        timeLeft -= timeout.in(Seconds);
        // If we get negative time, our guess was very wrong so we'll return a blank string
        return timeLeft > 0 ? " | " + (approx.get() ? "~" : "") + Mathf.round(timeLeft, 1) + "s" : "";
    }

    /**
     * Removes whatever task is at the given queue position.
     * <p>
     * Note: this will remove the index and shift all other tasks down, meaning that
     * tasks being added/removed will affect the index of the task you want to remove
     *
     * @param taskIndex the array index to be removed, starting from 0
     */
    public final void removeAtIndex(int taskIndex) {
        synchronized (tasks) {
            if (taskIndex < 0 || taskIndex >= tasks.size())
                throw new IllegalArgumentException("Cannot remove task at index " + taskIndex + ", out of bounds");

            /*
             * In the words of the great Lucas Bubner:
             *      You've made an iterator for all those tasks
             *      which is the goofinator car that can drive around your array
             *      calling .next() on your car will move it one down the array
             *      then if you call .remove() on your car it will remove the element wherever it is
             */
            Iterator<Task> iterator = tasks.iterator();

            int counter = 0;
            while (iterator.hasNext()) {
                iterator.next();

                if (counter == taskIndex) {
                    iterator.remove();
                    telemetry.log("<font color='gray'>auto:</font> task at index % -> removed", taskIndex);
                    taskCount--;
                    break;
                }
                counter++;
            }
        }
    }

    /**
     * Remove a task from the queue.
     * <p>
     * This assumes that the overhead OpMode has instance control over the task, as this method
     * will search for an object reference to the task and remove it from the queue
     *
     * @param task the task to be removed
     */
    public final void remove(@NonNull Task task) {
        synchronized (tasks) {
            if (tasks.contains(task)) {
                tasks.remove(task);
                telemetry.log("<font color='gray'>auto:</font> task %<i>(t=%)</i> -> removed", task, getTaskTimeout(task));
                taskCount--;
            } else {
                telemetry.log("<font color='gray'>auto:</font> task %<i>(t=%)</i> -> <font color='yellow'>not found</font>", task, getTaskTimeout(task));
            }
        }
    }

    /**
     * Removes the last task in the task queue.
     */
    public final void removeLast() {
        synchronized (tasks) {
            tasks.removeLast();
        }
        taskCount--;
        telemetry.log("<font color='gray'>auto:</font> task at index % -> removed", taskCount + 1);
    }

    /**
     * Removes the first task in the task queue.
     */
    public final void removeFirst() {
        synchronized (tasks) {
            tasks.removeFirst();
        }
        taskCount--;
        telemetry.log("<font color='gray'>auto:</font> task at index 0 -> removed");
    }

    private void checkTaskForDependency(Task task) {
        task.getDependency().ifPresent((s) -> {
            if (!updatedSubsystems.contains(s)) {
                Dbg.warn(getClass(), "Task % has a dependency on %, but it is not being updated by the AutonomousBunyipsOpMode. This is due to a call to use() that is not including this subsystem. Please ensure this is intended behaviour. A clearer alternative is to disable() the subsystem(s) you don't wish to update.", task, s);
                telemetry.log(Text.html().color("yellow", "auto: ").text("dependency % for task % has not been updated"), s, task);
            }
        });
    }

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised (if applicable), and to assign OpModes via {@link #setOpModes}.
     * You may also add specific tasks to the queue here, but it is recommended to use {@link #setInitTask}
     * or {@link #onReady(RefCell)} instead.
     * <p>
     * Override this method to use it.
     */
    protected void onInitialise() {
        // no-op (>= v7.0.0)
    }

    /**
     * Call to define your user selection "sub-OpModes". If you list any, then the user will be prompted to select
     * from these options during initialisation asynchronously. If you return null or don't call this method,
     * then the user will not be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * <p>
     * Review the {@link UserSelection} class and wiki documentation for more information regarding this feature,
     * including chaining and what to expect regarding the runtime of this class.
     * <p>
     * By setting OpModes here, the {@link #onReady(RefCell)} parameter will be of type {@link T} and you can cast
     * it without needing to check.
     */
    @SafeVarargs
    protected final <T> UserSelection<?> setOpModes(@Nullable T... selectableOpModes) {
        if (selectableOpModes == null) return null;
        selections = selectableOpModes;
        // This will run asynchronously later, and the callback will be called when the user has selected an OpMode
        // An empty selections array will cause an immediate return of the callback on execution, handled on init
        userSelection = new UserSelection<>(this::callback, selectableOpModes);
        // Note we already downcast to a wildcard since T can be modified if we're using a prebuilt
        // starting position (legacy), and we also expect a wildcard type on onReady.
        return userSelection; // Allow chaining if the user wants to disable v7.0.0 features
    }

    /**
     * Called when the OpMode is ready to process tasks.
     * This will happen when the user has selected an OpMode, or if {@link #setOpModes(Object...)} was not called,
     * in which case it will run immediately after {@code static_init} has completed.
     * This is where you should add your tasks to the run queue.
     * <p>
     * For versions >=7.0.0, you can access the {@code selectedButton} field through {@code UserSelection.getLastSelectedButton()}.
     *
     * @param selectedOpMode the sub-OpMode selection by the user via {@link #setOpModes}, if applicable. Will be {@code null} if the user does not select *any* sub-OpMode (and options were available).
     *                       Will be an empty reference if {@link #setOpModes(Object...)} returned null/was not called (no OpModes to select). If you have used array chaining for your selections,
     *                       this parameter will be of type `T[]` or `Collection<T>` and contains an array with the results from each layer (such that if you had two arrays in, you would
     *                       get one array back with a size of two, elements being the selections from array one and two). If the chaining selection ends halfway through, the array is filled to length with null values.
     * @see #add(Task)
     */
    protected abstract void onReady(@Nullable RefCell<?> selectedOpMode);

    /**
     * Override this method to add extra code to the activeLoop, which will be run before the task queue is processed.
     */
    protected void periodic() {
        // no-op
    }

    /**
     * Finish actions that can be taken following the completion of all tasks in the queue.
     *
     * @since 7.0.0
     */
    public enum OnTasksDone {
        /**
         * Executes {@link #finish()} and automatically halts all hardware via the {@link #safeHaltHardware()} method
         * when all tasks are completed.
         * This is the default behaviour.
         */
        FINISH_OPMODE,
        /**
         * Executes {@link #finish()}, but does not safe halt hardware when all tasks are completed. This is useful for
         * situations where servos should remain powered for the rest of the OpMode, and motor powers left at their last value.
         * This mode is not appropriate for controls that require an active loop, as system controllers will receive no updates.
         */
        FINISH_OPMODE_NO_HALT_HARDWARE,
        /**
         * Continue running the OpMode, allowing default tasks on subsystems and periodic to continue execution when all tasks are completed.
         * This is most appropriate for PID loops that need to continue, such as holding a position until the
         * rest of the OpMode is completed.
         */
        CONTINUE_EXECUTION
    }

    /**
     * Priority representation for building tasks.
     */
    public enum TaskPriority {
        /**
         * Add the task to the end of the queue after the onReady() init callback has fired
         */
        LAST,
        /**
         * Add the task to the queue immediately (default)
         */
        NORMAL,
        /**
         * Add the task to the front of the queue after the onReady() init callback has fired
         */
        FIRST//® Tech Challenge
    }
}
