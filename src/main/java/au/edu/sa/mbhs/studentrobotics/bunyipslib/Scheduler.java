package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.RunTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.OnceTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Scheduler and command plexus for use with the BunyipsLib task system in TeleOp.
 *
 * @author Lucas Bubner, 2024
 * @see CommandBasedBunyipsOpMode
 * @since 1.0.0-pre
 */
public class Scheduler extends BunyipsComponent {
    private static final ArrayList<String> reports = new ArrayList<>();
    private static boolean isMuted = false;
    private final ArrayList<BunyipsSubsystem> subsystems = new ArrayList<>();
    private final ArrayList<ScheduledTask> allocatedTasks = new ArrayList<>();

    /**
     * Create a new scheduler and reset static fields.
     */
    public Scheduler() {
        isMuted = false;
        reports.clear();
    }

    /**
     * Used internally by subsystems and tasks to report their running status statically.
     * This method is not intended for use by the user.
     *
     * @param className     The class name of the subsystem or context.
     * @param isDefaultTask Whether this task is a default task.
     * @param taskName      The name of the task.
     * @param deltaTimeSec  The time this task has been running in seconds
     * @param timeoutSec    The time this task is allowed to run in seconds, 0.0 if indefinite
     */
    public static void addTaskReport(String className, boolean isDefaultTask, String taskName, double deltaTimeSec, double timeoutSec) {
        if (isMuted) return;
        String report = Text.format(
                "<small><b>%</b>% <font color='gray'>|</font> <b>%</b> -> %",
                className,
                isDefaultTask ? " (d.)" : "",
                taskName,
                deltaTimeSec
        );
        report += timeoutSec == 0.0 ? "s" : "/" + timeoutSec + "s";
        reports.add(report + "</small>");
    }

    /**
     * Get all allocated tasks.
     */
    public ScheduledTask[] getAllocatedTasks() {
        return allocatedTasks.toArray(new ScheduledTask[0]);
    }

    /**
     * Get all subsystems attached to the scheduler.
     */
    public BunyipsSubsystem[] getManagedSubsystems() {
        return subsystems.toArray(new BunyipsSubsystem[0]);
    }

    /**
     * Add subsystems to the scheduler. This will ensure the update() method of the subsystems is called, and that
     * commands can be scheduled on these subsystems.
     * This is <b>REQUIRED</b> to be called if using a base implementation of Scheduler. If you are using a
     * {@link CommandBasedBunyipsOpMode}, see the {@code useSubsystems()} method or rely on the automatic features during
     * construction that will add subsystems at construction with no need to call this method.
     * <p>
     * The base implementation of Scheduler does not access this implicit construction for finer-grain control for
     * implementations that don't want this behaviour.
     *
     * @param dispatch The subsystems to add.
     */
    public void addSubsystems(BunyipsSubsystem... dispatch) {
        subsystems.addAll(Arrays.asList(dispatch));
        if (subsystems.isEmpty())
            Dbg.warn(getClass(), "Caution: No subsystems were added for the Scheduler to update.");
        else
            Dbg.logv(getClass(), "Added % subsystem(s) to update.", dispatch.length);
    }

    /**
     * Disable all subsystems attached to the Scheduler.
     */
    public void disable() {
        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.disable();
        }
    }

    /**
     * Enable all subsystems attached to the Scheduler, unless they failed from null assertion.
     */
    public void enable() {
        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.enable();
        }
    }

    /**
     * Mute Scheduler telemetry.
     */
    public void mute() {
        isMuted = true;
    }

    /**
     * Unmute Scheduler telemetry.
     */
    public void unmute() {
        isMuted = false;
    }

    /**
     * Run the scheduler. This will run all subsystems and tasks allocated to the scheduler.
     * This should be called in the {@code activeLoop()} method of the {@link BunyipsOpMode}, and is automatically called
     * in {@link CommandBasedBunyipsOpMode}.
     */
    public void run() {
        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.update();
        }

        if (!isMuted) {
            opMode(o -> {
                // Task count will account for tasks on subsystems that are not IdleTasks
                int taskCount = (int) (allocatedTasks.size() + subsystems.size() - subsystems.stream().filter(BunyipsSubsystem::isIdle).count());
                o.telemetry.add("\nManaging % task% (%s, %c) on % subsystem%",
                        taskCount,
                        taskCount == 1 ? "" : "s",
                        allocatedTasks.stream().filter(task -> task.taskToRun.hasDependency()).count() + taskCount - allocatedTasks.size(),
                        allocatedTasks.stream().filter(task -> !task.taskToRun.hasDependency()).count(),
                        subsystems.size(),
                        subsystems.size() == 1 ? "" : "s"
                );
                for (String item : reports) {
                    if (item.contains("IdleTask")) continue;
                    o.telemetry.add(item);
                }
                for (ScheduledTask task : allocatedTasks) {
                    if (task.taskToRun.hasDependency() // Whether the task is never run from the Scheduler (and task reports will come from the reports array)
                            || !task.taskToRun.isRunning() // Whether this task is actually running
                            || task.taskToRun.isMuted() // Whether the task has declared itself as muted
                    ) {
                        continue;
                    }
                    double deltaTime = Mathf.round(task.taskToRun.getDeltaTime().in(Seconds), 1);
                    o.telemetry.add(
                            "<small><b>Scheduler</b> (c.) <font color='gray'>|</font> <b>%</b> -> %</small>",
                            task.taskToRun,
                            deltaTime == 0.0 ? "active" : deltaTime + "s"
                    );
                }
            });
            reports.clear();
        }

        for (ScheduledTask task : allocatedTasks) {
            boolean condition = task.runCondition.getAsBoolean();
            if (condition || task.taskToRun.isRunning()) {
                if (!task.taskToRun.hasDependency()) {
                    if (task.stopCondition.getAsBoolean()) {
                        // Finish now as we should do nothing with this task
                        task.taskToRun.finishNow();
                        continue;
                    }
                    // This is a non-command task, run it now as it will not be run by any subsystem
                    task.taskToRun.run();
                    // Debouncing should not auto-reset the task if it is completed
                    if (task.taskToRun.pollFinished() && !task.debouncing) {
                        // Reset the task as it is not attached to a subsystem and will not be reintegrated by one
                        task.taskToRun.reset();
                    }
                    continue;
                }
                // This task must have a dependency, set the current task of the subsystem that depends on it
                // Tasks may only have one subsystem dependency, where this dependency represents where the task
                // will be executed by the scheduler.
                assert task.taskToRun.getDependency().isPresent();
                if (task.stopCondition.getAsBoolean()) {
                    // Finish handler will be called on the subsystem
                    task.taskToRun.finish();
                    continue;
                }
                if (task.taskToRun.isFinished() && task.debouncing) {
                    // Don't requeue if debouncing
                    continue;
                }
                task.taskToRun.getDependency().get().setCurrentTask(task.taskToRun);
            } else if (task.taskToRun.isFinished()) {
                task.taskToRun.reset();
            }
        }
    }

    /**
     * Create a new controller button trigger creator.
     *
     * @param user The Controller instance to use.
     * @return The controller button trigger creator.
     */
    public ControllerTriggerCreator when(Controller user) {
        return new ControllerTriggerCreator(user);
    }

    /**
     * Create a new controller button trigger creator for the driver.
     *
     * @return The controller button trigger creator.
     */
    public ControllerTriggerCreator driver() {
        return new ControllerTriggerCreator(require(opMode).gamepad1);
    }

    /**
     * Create a new controller button trigger creator for the operator.
     *
     * @return The controller button trigger creator.
     */
    public ControllerTriggerCreator operator() {
        return new ControllerTriggerCreator(require(opMode).gamepad2);
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    public ScheduledTask when(BooleanSupplier condition) {
        if (condition instanceof Condition) {
            return new ScheduledTask((Condition) condition);
        }
        return new ScheduledTask(new Condition(condition));
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a rising-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    public ScheduledTask whenRising(BooleanSupplier condition) {
        return new ScheduledTask(new Condition(condition, Condition.Edge.RISING));
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a falling-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    public ScheduledTask whenFalling(BooleanSupplier condition) {
        return new ScheduledTask(new Condition(condition, Condition.Edge.FALLING));
    }

    /**
     * Run a task always. This is the same as calling {@code .when(() -> true)}.
     *
     * @return Task scheduling builder
     */
    public ScheduledTask always() {
        return new ScheduledTask(new Condition(() -> true));
    }

    private static class ControllerButtonBind extends Condition {
        protected final Controls button;
        protected final Controller controller;

        ControllerButtonBind(Controller controller, Controls button, Edge edge) {
            super(() -> controller.get(button), edge);
            this.button = button;
            this.controller = controller;
        }

        @NonNull
        @Override
        public String toString() {
            return "Button:" + controller.getUser().toString() + "->" + button.toString();
        }
    }

    private static class ControllerAxisThreshold extends Condition {
        private final Controls.Analog axis;

        ControllerAxisThreshold(Controller user, Controls.Analog axis, Predicate<? super Float> threshold, Edge edge) {
            super(() -> threshold.test(user.get(axis)), edge);
            this.axis = axis;
        }

        @NonNull
        @Override
        public String toString() {
            return "Axis:" + axis.toString();
        }
    }

    /**
     * Controller trigger creator.
     */
    public class ControllerTriggerCreator {
        private final Controller user;

        private ControllerTriggerCreator(Controller user) {
            this.user = user;
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        public ScheduledTask when(Controls.Analog axis, Predicate<? super Float> threshold) {
            return new ScheduledTask(new ControllerAxisThreshold(user, axis, threshold, Condition.Edge.ACTIVE));
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a rising-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        public ScheduledTask whenRising(Controls.Analog axis, Predicate<? super Float> threshold) {
            return new ScheduledTask(new ControllerAxisThreshold(user, axis, threshold, Condition.Edge.RISING));
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a falling-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        public ScheduledTask whenFalling(Controls.Analog axis, Predicate<? super Float> threshold) {
            return new ScheduledTask(new ControllerAxisThreshold(user, axis, threshold, Condition.Edge.FALLING));
        }

        /**
         * Run a task when a controller button is held.
         * This condition will be evaluated continuously.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        public ScheduledTask whenHeld(Controls button) {
            return new ScheduledTask(new ControllerButtonBind(user, button, Condition.Edge.ACTIVE));
        }

        /**
         * Run a task when a controller button is pressed (will run once when pressing the desired input).
         * This is the same as rising-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        public ScheduledTask whenPressed(Controls button) {
            return new ScheduledTask(new ControllerButtonBind(user, button, Condition.Edge.RISING));
        }

        /**
         * Run a task when a controller button is released (will run once letting go of the desired input).
         * This is the same as falling-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        public ScheduledTask whenReleased(Controls button) {
            return new ScheduledTask(new ControllerButtonBind(user, button, Condition.Edge.FALLING));
        }
    }

    /**
     * A task that will run when a condition is met.
     */
    public class ScheduledTask {
        protected final Condition originalRunCondition;
        protected final BooleanSupplier runCondition;
        private final ArrayList<BooleanSupplier> and = new ArrayList<>();
        private final ArrayList<BooleanSupplier> or = new ArrayList<>();
        protected Task taskToRun;
        protected boolean debouncing;
        protected BooleanSupplier stopCondition = () -> false;

        private boolean isTaskMuted = false;

        /**
         * Create and allocate a new conditional task. This will automatically be added to the scheduler.
         *
         * @param originalRunCondition The condition to start running the task.
         */
        public ScheduledTask(Condition originalRunCondition) {
            // Run the task if the original expression is met,
            // and all AND conditions are met, or any OR conditions are met
            runCondition = () -> originalRunCondition.getAsBoolean()
                    && and.stream().allMatch(BooleanSupplier::getAsBoolean)
                    || or.stream().anyMatch(BooleanSupplier::getAsBoolean);
            this.originalRunCondition = originalRunCondition;
            allocatedTasks.add(this);
        }

        /**
         * Queue a task when the condition is met.
         * This task will run (and self-reset if finished) for as long as the condition is met.
         * <p>
         * Note this means that the task provided will run from start-to-finish when the condition is true, which means
         * it <i>won't execute exclusively while the condition is met</i>, rather have the capability to be started when
         * the condition is met. This means continuous iterations of a true condition will try to keep this task queued
         * at all times, resetting the task internally when it is completed. Keep this in mind if working with
         * looping/long tasks, as you might experience runaway tasks.
         * See {@link #finishingIf} for fine-grain "run exclusively if" control.
         * <p>
         * This method can only be called once per ConditionalTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        public ScheduledTask run(Task task) {
            if (taskToRun != null) {
                throw new EmergencyStop("A run(Task) method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.");
            }
            taskToRun = task;
            if (isTaskMuted)
                taskToRun.withMutedReports();
            return this;
        }

        /**
         * Implicitly make a new RunTask to run once the condition is met.
         * This callback will run repeatedly while the condition is met.
         * <p>
         * This method can only be called once per ConditionalTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an RunTask.
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        public ScheduledTask run(Runnable runnable) {
            return run(new RunTask(runnable));
        }

        /**
         * Queue a task when the condition is met, debouncing the task from queueing more than once the condition is met.
         * This effectively does the same as {@link #run}, however only a single queue is permitted per rising edge.
         * <p>
         * This method can only be called once per ConditionalTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        public ScheduledTask runOnce(Task task) {
            debouncing = true;
            return run(task);
        }

        /**
         * Implicitly make a new RunTask to run once the condition is met, debouncing the task from queueing more than once the condition is met.
         * This effectively will run this code block once when the condition is met at the rising edge.
         * <p>
         * This method can only be called once per ConditionalTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an RunTask.
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        public ScheduledTask runOnce(Runnable runnable) {
            return runOnce(new RunTask(runnable));
        }

        /**
         * Mute this task from being a part of the Scheduler report.
         *
         * @return Current builder for additional task parameters
         */
        public ScheduledTask muted() {
            if (taskToRun != null) {
                taskToRun.withMutedReports();
            }
            isTaskMuted = true;
            return this;
        }

        /**
         * Chain an AND condition to the current conditional task.
         * Will be evaluated after the controller condition, and before the OR conditions.
         *
         * @param condition The AND condition to chain.
         * @return Current builder for additional task parameters
         */
        public ScheduledTask andIf(BooleanSupplier condition) {
            and.add(condition);
            return this;
        }

        /**
         * Chain an OR condition to the current conditional task.
         * Will be evaluated after the controller and AND conditions.
         *
         * @param condition The OR condition to chain.
         * @return Current builder for additional task parameters
         */
        public ScheduledTask orIf(BooleanSupplier condition) {
            or.add(condition);
            return this;
        }

        /**
         * Run a task assigned to in run() in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        public ScheduledTask in(Measure<Time> interval) {
            originalRunCondition.withActiveDelay(interval);
            return this;
        }

        /**
         * Run a task assigned to in run() in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        // Kotlin interop, as in is a reserved keyword
        public ScheduledTask inTime(Measure<Time> interval) {
            return in(interval);
        }

        /**
         * Run the task assigned to in run() until this condition is met. Once this condition is met, the task will
         * be forcefully stopped and the scheduler will move on. This is useful for continuous tasks.
         * If this method is called multiple times, an OR condition will be composed with the last condition.
         *
         * @param condition The condition to stop the task. Note the task will be auto-stopped if it finishes by itself,
         *                  this condition simply allows for an early finish if this condition is met.
         * @return Current builder for additional task parameters
         */
        public ScheduledTask finishingIf(BooleanSupplier condition) {
            // Use prev to avoid a stack overflow
            BooleanSupplier prev = stopCondition;
            stopCondition = prev == null
                    ? condition
                    : () -> prev.getAsBoolean() || condition.getAsBoolean();
            return this;
        }

        @NonNull
        @Override
        public String toString() {
            Text.Builder out = Text.builder();
            out.append(taskToRun.hasDependency() ? "Scheduling " : "Running ")
                    .append("'")
                    .append(taskToRun.toString())
                    .append("'");
            double timeout = taskToRun.getTimeout().in(Seconds);
            if (timeout * 1000.0 > OnceTask.EPSILON_MS) {
                out.append(" (t=").append(timeout).append("s)");
            }
            if (taskToRun.isOverriding())
                out.append(" (overriding)");
            if (originalRunCondition instanceof ControllerButtonBind) {
                ControllerButtonBind handler = (ControllerButtonBind) originalRunCondition;
                out.append(" when GP")
                        .append(handler.controller.getUser() == GamepadUser.ONE ? 1 : 2)
                        .append("->")
                        .append(handler.button)
                        .append(" is ")
                        .append(handler.edge);
            } else {
                out.append(" when ")
                        .append(originalRunCondition.toString().replace(BuildConfig.LIBRARY_PACKAGE_NAME + ".Scheduler", ""))
                        .append(" is true");
            }
            Measure<Time> delay = originalRunCondition.getActiveDelay();
            if (delay.magnitude() > 0) {
                out.append(", delayed ")
                        .append(Mathf.round(delay.in(Seconds), 1))
                        .append("s");
            }
            out.append(!and.isEmpty() ? ", " + and.size() + " extra AND condition(s)" : "")
                    .append(!or.isEmpty() ? ", " + or.size() + " extra OR condition(s)" : "")
                    .append(debouncing ? ", debouncing" : "")
                    .append(isTaskMuted || taskToRun.isMuted() ? ", task status muted" : "");
            return out.toString();
        }
    }
}
