package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.DeferredTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingConfiguration;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;

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
     * Used for tasks that have no timeout to generate a "estimate to OpMode completion" metric.
     * Purely visual, and does not affect the actual task (hence why this field is not exposed to FtcDashboard).
     */
    public static double INFINITE_TASK_ASSUMED_DURATION_SECONDS = 5.0;
    private final ArrayList<Reference<?>> opModes = new ArrayList<>();
    private final ConcurrentLinkedDeque<Task> tasks = new ConcurrentLinkedDeque<>();
    // Pre and post queues cannot have their tasks removed, so we can rely on their .size() methods
    private final ConcurrentLinkedDeque<Task> postQueue = new ConcurrentLinkedDeque<>();
    private final ConcurrentLinkedDeque<Task> preQueue = new ConcurrentLinkedDeque<>();
    @NonNull
    private HashSet<BunyipsSubsystem> updatedSubsystems = new HashSet<>();
    private int taskCount;
    private UserSelection<Reference<?>> userSelection;
    private int currentTask = 1;
    private volatile boolean safeToAddTasks;
    private volatile boolean callbackReceived;
    private boolean hardwareStopOnFinish = true;

    private void callback(@Nullable Reference<?> selectedOpMode) {
        // Safety as the OpMode may not be running anymore (due to it being on another thread)
        if (isStopRequested())
            return;

        safeToAddTasks = true;

        Controls selectedButton = userSelection != null ? userSelection.getSelectedButton() : Controls.NONE;
        Exceptions.runUserMethod(() -> onReady(selectedOpMode, selectedButton), this);
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
        out.append("[AutonomousBunyipsOpMode] onReady() called | %% task(s) queued%\n",
                userSelection != null ? "usr: " + Text.removeHtml(String.valueOf(selectedOpMode)) + " | " : "",
                taskCount,
                timeLeft.isEmpty() ? "" : timeLeft + " to complete"
        );
        for (Task task : tasks) {
            out.append("   -> %\n", task.toVerboseString());
        }
        Dbg.logd(out.toString());
    }

    @Override
    protected final void onInit() {
        // Run user-defined hardware initialisation
        Exceptions.runUserMethod(this::onInitialise, this);
        if (updatedSubsystems.isEmpty()) {
            // We might be using an implicit subsystem initialisation schema, look for static instances instead
            updatedSubsystems = BunyipsSubsystem.getInstances();
        }
        if (updatedSubsystems.isEmpty()) {
            Dbg.warn(getClass(), "Caution: No subsystems are attached to AutonomousBunyipsOpMode that can be updated.");
        }

        // Convert user defined OpModeSelections to varargs
        Reference<?>[] varargs = opModes.toArray(new Reference[0]);
        if (varargs.length == 0) {
            opModes.add(Reference.empty());
        }
        if (varargs.length > 1) {
            // Run task allocation if OpModeSelections are defined
            // This will run asynchronously, and the callback will be called
            // when the user has selected an OpMode
            userSelection = new UserSelection<>(this::callback, varargs);
            Threads.start(userSelection);
        } else {
            // There are no OpMode selections, so just run the callback with the default OpMode
            callback(opModes.get(0));
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
            // UserSelection will internally check opMode.isInInit() to see if it should terminate itself
            // but we should wait here until it has actually terminated
            Threads.waitFor(userSelection, true);
        }
        // Busy wait here until onReady() has processed and the callback is fully joined
        // This is safe to do as there are no main thread operations left to run
        while (!callbackReceived && !isStopRequested()) {
            sleep(1);
        }
    }

    @Override
    protected final void activeLoop() {
        // Run any code defined by the user
        Exceptions.runUserMethod(this::periodic, this);

        // Update all subsystems which may also contain user routines
        // This also ensures the subsystems are ready to accept incoming tasks
        for (BunyipsSubsystem subsystem : updatedSubsystems) {
            subsystem.update();
        }

        // Run the queue of tasks
        synchronized (tasks) {
            Task currentTask = tasks.peekFirst();
            if (currentTask == null) {
                telemetry.log("<font color='gray'>auto:</font> tasks done -> finishing");
                finish(hardwareStopOnFinish);
                return;
            }

            telemetry.setOverheadSubtitle(
                    Text.format("<small><font color='aqua'>Running task <b>%/%</b></font> | %%</small>",
                            this.currentTask, taskCount, currentTask, getApproximateTimeLeft())
            );

            try {
                // AutonomousBunyipsOpMode is handling all task completion checks, manual checks not required
                if (currentTask.pollFinished()) {
                    tasks.removeFirst();
                    double runTime = currentTask.getDeltaTime().in(Seconds);
                    Dbg.logd("[AutonomousBunyipsOpMode] task %/% (%) finished%", this.currentTask, taskCount, currentTask, runTime != 0 ? " -> " + runTime + "s" : "");
                    this.currentTask++;
                }

                currentTask.run();
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
     * Call to disable the automatic stopping of the hardware when the OpMode finishes after no tasks are left.
     * This does not impact the automated stopping of the hardware when the OpMode is requested to stop.
     */
    public final void disableHardwareStopOnFinish() {
        hardwareStopOnFinish = false;
    }

    /**
     * Call to manually add the subsystems that should be managed by AutonomousBunyipsOpMode. Using this method will override
     * the automatic collection of {@link BunyipsSubsystem}s, and allows you to determine which subsystems will be managed for
     * this OpMode.
     * <p>
     * For most cases, using this method is not required and all you need to do is construct your subsystems and they
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
            tasks.add(newTask);
        }
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added %/%", newTask, getTaskTimeout(newTask), taskCount, taskCount);
        return newTask;
    }

    /**
     * Call to add {@link Task} instances that will be executed sequentially during the active loop.
     * This task will be internally wrapped into a {@link DeferredTask}.
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
     * Implicitly constructs a new {@link Lambda} to add to the run queue.
     *
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda run(@NonNull Runnable runnable) {
        return add(new Lambda(runnable));
    }

    /**
     * Implicitly constructs a new {@link Lambda} to add to the run queue.
     *
     * @param name     the name of the task
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda run(@NonNull String name, @NonNull Runnable runnable) {
        Lambda task = new Lambda(runnable);
        task.withName(name);
        return add(task);
    }

    /**
     * Implicitly constructs a new {@link WaitTask} to add to the run queue.
     *
     * @param duration the duration to wait
     * @return the added {@link WaitTask}
     */
    @NonNull
    public final WaitTask wait(@NonNull Measure<Time> duration) {
        return add(new WaitTask(duration));
    }

    /**
     * Implicitly constructs a new {@link WaitTask} to add to the run queue.
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
            tasks.add(newTask);
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
     * Insert an implicit RunTask at a specific index in the queue.
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
     * Insert an implicit RunTask at a specific index in the queue.
     *
     * @param index    the index to insert the task at, starting from 0
     * @param name     the name of the task
     * @param runnable the code to add to the run queue to run once
     * @return the added {@link Lambda}
     */
    @NonNull
    public final Lambda addAtIndex(int index, @NonNull String name, @NonNull Runnable runnable) {
        Lambda task = new Lambda(runnable);
        task.withName(name);
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
     * Add a task to the run queue, but after {@link #onReady(Reference, Controls)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(Reference, Controls)}.
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
     * Add a task to the run queue, but after {@link #onReady(Reference, Controls)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(Reference, Controls)}.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
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
     * asynchronously with user input in {@link #onReady(Reference, Controls)}.
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
     * asynchronously with user input in {@link #onReady(Reference, Controls)}.
     * <p>
     * This task will be internally wrapped into a {@link DeferredTask}.
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
        Measure<Time> timeout = task.getTimeout();
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
            // as they are completely out of our control and we don't even know how they function
            Measure<Time> timeout = task.getTimeout();
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
        Measure<Time> timeout = curr.getDeltaTime();
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
     * This is where your hardware should be initialised. You may also add specific tasks to the queue
     * here, but it is recommended to use {@link #setInitTask} or {@link #onReady(Reference, Controls)} instead.
     */
    protected abstract void onInitialise();

    /**
     * Call to define your OpModeSelections, if you list any, then the user will be prompted to select
     * an OpMode before the OpMode begins. If you return null/don't call this method, then the user will not
     * be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * This determines what is to be used in the parameters of {@link #onReady(Reference, Controls)}.
     *
     * <pre>{@code
     *     setOpModes(
     *             "GO_PARK",
     *             "GO_SHOOT",
     *             "GO_SHOOT_AND_PARK",
     *             "SABOTAGE_ALLIANCE"
     *     );
     *     // See the StartingConfiguration class for advanced builder patterns of robot starting positions,
     *     // which is the recommended way to define OpModes (OpModes themselves define objectives, not positions)
     * }</pre>
     */
    protected final void setOpModes(@Nullable List<Object> selectableOpModes) {
        if (selectableOpModes == null) return;
        setOpModes(selectableOpModes.toArray(new Object[0]));
    }


    /**
     * Call to define your OpModeSelections, if you list any, then the user will be prompted to select
     * an OpMode before the OpMode begins. If you return null, then the user will not
     * be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * <pre>{@code
     *     setOpModes(
     *             "GO_PARK",
     *             "GO_SHOOT",
     *             "GO_SHOOT_AND_PARK",
     *             "SABOTAGE_ALLIANCE"
     *     );
     *     // See the StartingConfiguration class for advanced builder patterns of robot starting positions,
     *     // which is the recommended way to define OpModes (OpModes themselves define objectives, not positions)
     * }</pre>
     */
    protected final void setOpModes(@Nullable Object... selectableOpModes) {
        if (selectableOpModes == null) return;
        opModes.clear();
        for (Object selectableOpMode : selectableOpModes) {
            if (selectableOpMode instanceof Reference<?>) {
                opModes.add((Reference<?>) selectableOpMode);
            } else if (selectableOpMode instanceof StartingConfiguration.Builder.PrebuiltPosition) {
                // Preemptive catch for non-built StartingConfigurations which are a common use case
                // No point in throwing errors for the little stuff we can solve here now
                opModes.add(new Reference<>(((StartingConfiguration.Builder.PrebuiltPosition) selectableOpMode).build()));
            } else {
                opModes.add(new Reference<>(selectableOpMode));
            }
        }
    }

    /**
     * Called when the OpMode is ready to process tasks.
     * This will happen when the user has selected an OpMode, or if {@link #setOpModes(Object...)} returned null,
     * in which case it will run immediately after {@code static_init} has completed.
     * This is where you should add your tasks to the run queue.
     *
     * @param selectedOpMode the OpMode selected by the user, if applicable. Will be NULL if the user does not select an OpMode (and OpModes were available).
     *                       Will be an empty reference if {@link #setOpModes(Object...)} returned null (no OpModes to select).
     * @param selectedButton the button selected by the user. Will be Controls.NONE if no selection is made or given.
     * @see #add(Task)
     */
    protected abstract void onReady(@Nullable Reference<?> selectedOpMode, @NonNull Controls selectedButton);

    /**
     * Override to this method to add extra code to the activeLoop, which will be run before
     * the task queue is processed.
     */
    protected void periodic() {
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
