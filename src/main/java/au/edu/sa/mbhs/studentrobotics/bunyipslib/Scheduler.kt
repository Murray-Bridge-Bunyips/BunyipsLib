package au.edu.sa.mbhs.studentrobotics.bunyipslib

import android.annotation.SuppressLint
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg.logv
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg.warn
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls.Analog
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser
import java.util.function.BooleanSupplier

/**
 * Scheduler and command plexus for use with the BunyipsLib task system in TeleOp.
 *
 * @author Lucas Bubner, 2024
 * @see CommandBasedBunyipsOpMode
 * @since 1.0.0-pre
 */
class Scheduler : BunyipsComponent() {
    private val subsystems = ArrayList<BunyipsSubsystem>()
    private val allocatedTasks = ArrayList<ScheduledTask>()

    init {
        isMuted = false
    }

    /**
     * Get all allocated tasks.
     */
    fun getAllocatedTasks(): Array<ScheduledTask> {
        return allocatedTasks.toTypedArray<ScheduledTask>()
    }

    val managedSubsystems: Array<BunyipsSubsystem>
        /**
         * Get all subsystems attached to the scheduler.
         */
        get() = subsystems.toTypedArray<BunyipsSubsystem>()

    /**
     * Add subsystems to the scheduler. This will ensure the `update()` method of the subsystems is called, and that
     * commands can be scheduled on these subsystems.
     * This is **REQUIRED** to be called if using a base implementation of Scheduler. If you are using a
     * [CommandBasedBunyipsOpMode], see the [use] method or rely on the automatic features during
     * construction that will add subsystems at construction with no need to call this method.
     *
     * The base implementation of Scheduler does not access this implicit construction for finer-grain control for
     * implementations that don't want this behaviour.
     *
     * @param dispatch The subsystems to add.
     */
    fun addSubsystems(vararg dispatch: BunyipsSubsystem) {
        subsystems.addAll(listOf(*dispatch))
        if (subsystems.isEmpty()) warn(javaClass, "Caution: No subsystems were added for the Scheduler to update.")
        else logv(javaClass, "Added % subsystem(s) to update.", dispatch.size)
    }

    /**
     * Disable all subsystems attached to the Scheduler.
     */
    fun disable() {
        for (subsystem in subsystems) {
            subsystem.disable()
        }
    }

    /**
     * Enable all subsystems attached to the Scheduler, unless they failed from null assertion.
     */
    fun enable() {
        for (subsystem in subsystems) {
            subsystem.enable()
        }
    }

    /**
     * Mute Scheduler telemetry.
     */
    fun mute() {
        isMuted = true
    }

    /**
     * Unmute Scheduler telemetry.
     */
    fun unmute() {
        isMuted = false
    }

    /**
     * Run the scheduler. This will run all subsystems and tasks allocated to the scheduler.
     * This should be called in the `activeLoop()` method of the [BunyipsOpMode], and is automatically called
     * in [CommandBasedBunyipsOpMode].
     */
    fun run() {
        for (subsystem in subsystems) {
            subsystem.update()
        }

        if (!isMuted) {
            opMode {
                // Task count will account for tasks on subsystems that are not IdleTasks, and also subsystem tasks
                val taskCount = (allocatedTasks.size - allocatedTasks.stream()
                    .filter { task -> task.taskToRun.dependency.isPresent }.count()
                        + subsystems.size - subsystems.stream().filter { s -> s.isIdle }.count())
                it.telemetry.add("\nManaging % task% (%s, %c) on % subsystem%",
                    taskCount,
                    if (taskCount == 1L) "" else "s",
                    allocatedTasks.stream().filter { task -> task.taskToRun.dependency.isPresent }
                        .count() + taskCount - allocatedTasks.size,
                    allocatedTasks.stream().filter { task -> !task.taskToRun.dependency.isPresent }.count(),
                    subsystems.size,
                    if (subsystems.size == 1) "" else "s"
                )
                for (subsystem in subsystems) {
                    val task = subsystem.currentTask
                    if (task == null || task is IdleTask) continue
                    var report = Text.format(
                        "<small><b>%</b>% <font color='gray'>|</font> <b>%</b> -> %",
                        subsystem,
                        if (subsystem.isRunningDefaultTask) " (d.)" else "",
                        task.toString().replace("$subsystem:", ""),
                        task.deltaTime to Seconds round 1
                    )
                    val timeoutSec = task.timeout to Seconds
                    report += if (timeoutSec == 0.0) "s" else "/" + timeoutSec + "s"
                    report += "</small>"
                    it.telemetry.add(report)
                }
                for (task in allocatedTasks) {
                    if (task.taskToRun.dependency.isPresent // Whether the task is never run from the Scheduler (and task reports were handled above)
                        || !task.taskToRun.isRunning // Whether this task is actually running
                        || task.muted // Whether the task has declared itself as muted
                    ) {
                        continue
                    }
                    val deltaTime = task.taskToRun.deltaTime to Seconds round 1
                    it.telemetry.add(
                        "<small><b>Scheduler</b> (c.) <font color='gray'>|</font> <b>%</b> -> %</small>",
                        task.taskToRun,
                        if (deltaTime == 0.0) "active" else deltaTime.toString() + "s"
                    )
                }
            }
        }

        for (task in allocatedTasks) {
            val condition = task.runCondition.invoke()
            if (task.stopCondition == null) task.stopCondition = { false }
            if (condition || task.taskToRun.isRunning) {
                if (!task.taskToRun.dependency.isPresent) {
                    if (task.stopCondition!!.invoke()) {
                        // Finish now as we should do nothing with this task
                        task.taskToRun.finishNow()
                        continue
                    }
                    // This is a non-command task, run it now as it will not be run by any subsystem
                    task.taskToRun.run()
                    // Debouncing should not auto-reset the task if it is completed
                    if (task.taskToRun.poll() && !task.debouncing) {
                        // Reset the task as it is not attached to a subsystem and will not be reintegrated by one
                        task.taskToRun.reset()
                    }
                    continue
                }
                // This task must have a dependency, set the current task of the subsystem that depends on it
                // Tasks may only have one subsystem dependency, where this dependency represents where the task
                // will be executed by the scheduler.
                assert(task.taskToRun.dependency.isPresent)
                if (task.stopCondition!!.invoke()) {
                    // Finish handler will be called on the subsystem
                    task.taskToRun.finish()
                    continue
                }
                if (task.taskToRun.isFinished && task.debouncing) {
                    // Don't requeue if debouncing
                    continue
                }
                task.taskToRun.dependency.get().setCurrentTask(task.taskToRun)
            } else if (task.taskToRun.isFinished && !task.debouncing) {
                task.taskToRun.reset()
            }
        }
    }

    /**
     * Unbind a task from the scheduler, based on the index of the task in the scheduler's allocated tasks.
     *
     * This can either be determined by the order in which the tasks were bound, or by the ID of the task via
     * the [ScheduledTask.id] property, which is the same thing.
     *
     * @param index The index of the task to unbind.
     * @throws IndexOutOfBoundsException If the index is out of bounds.
     */
    fun unbind(index: Int) {
        allocatedTasks.removeAt(index)
    }

    /**
     * Unbind a scheduled task from the scheduler.
     *
     * @param task The [ScheduledTask] to unbind.
     */
    fun unbind(task: ScheduledTask) {
        allocatedTasks.remove(task)
    }

    /**
     * Create a new controller trigger creator.
     *
     * For Kotlin users, calling this method can be done with the notation `` `when` ``
     * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
     * or by calling the alias `on`.
     *
     * @param user The Controller instance to use.
     * @return The controller trigger creator.
     */
    @SuppressLint("NoHardKeywords")
    fun `when`(user: Controller): ControllerTriggerCreator {
        return ControllerTriggerCreator(user)
    }

    /**
     * Create a new controller trigger creator.
     *
     * @param user The Controller instance to use.
     * @return The controller button trigger creator.
     */
    fun on(user: Controller): ControllerTriggerCreator {
        return ControllerTriggerCreator(user)
    }

    /**
     * Create a new controller trigger creator for the driver (gamepad 1).
     *
     * @return The controller trigger creator.
     */
    fun driver(): ControllerTriggerCreator {
        return ControllerTriggerCreator(require(opMode).gamepad1)
    }

    /**
     * Create a new controller trigger creator for gamepad 1 (driver).
     *
     * @return The controller trigger creator.
     */
    fun gp1() = driver()

    /**
     * Create a new controller trigger creator for the operator (gamepad 2).
     *
     * @return The controller trigger creator.
     */
    fun operator(): ControllerTriggerCreator {
        return ControllerTriggerCreator(require(opMode).gamepad2)
    }

    /**
     * Create a new controller trigger creator for gamepad 2 (operator).
     *
     * @return The controller trigger creator.
     */
    fun gp2() = operator()

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     *
     * For Kotlin users, calling this method can be done with the notation `` `when` ``
     * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
     * or by calling the alias `on`.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @SuppressLint("NoHardKeywords")
    fun `when`(condition: BooleanSupplier): ScheduledTask {
        if (condition is Condition) {
            return ScheduledTask(condition)
        }
        return ScheduledTask(Condition(condition))
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun on(condition: BooleanSupplier): ScheduledTask {
        return `when`(condition)
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a rising-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun whenRising(condition: BooleanSupplier): ScheduledTask {
        return ScheduledTask(Condition(Condition.Edge.RISING, condition))
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a falling-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun whenFalling(condition: BooleanSupplier): ScheduledTask {
        return ScheduledTask(Condition(Condition.Edge.FALLING, condition))
    }

    /**
     * Run a task always. This is the same as calling `.when(() -> true)`.
     *
     * @return Task scheduling builder
     */
    fun always(): ScheduledTask {
        return ScheduledTask(Condition { true })
    }

    private class ControllerButtonBind(val controller: Controller, val button: Controls, edge: Edge) :
        Condition(edge, { controller[button] }) {
        override fun toString(): String {
            return "Button:" + controller.user.toString() + "->" + button.toString()
        }
    }

    private class ControllerAxisThreshold(
        user: Controller,
        private val axis: Analog,
        threshold: (Float) -> Boolean,
        edge: Edge
    ) :
        Condition(edge, { threshold.invoke(user[axis]) }) {
        override fun toString(): String {
            return "Axis:$axis"
        }
    }

    /**
     * Controller trigger creator.
     */
    inner class ControllerTriggerCreator(private val user: Controller) {
        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * For Kotlin users, calling this method can be done with the notation `` `when` ``
         * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
         * or by calling the alias `on`.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        @SuppressLint("NoHardKeywords")
        fun `when`(axis: Analog, threshold: (Float) -> Boolean): ScheduledTask {
            return ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.ACTIVE))
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun on(axis: Analog, threshold: (Float) -> Boolean): ScheduledTask {
            return `when`(axis, threshold)
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun on(condition: Pair<Analog, (Float) -> Boolean>): ScheduledTask {
            return `when`(condition.first, condition.second)
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a rising-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun whenRising(axis: Analog, threshold: (Float) -> Boolean): ScheduledTask {
            return ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.RISING))
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a rising-edge detection.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun whenRising(condition: Pair<Analog, (Float) -> Boolean>): ScheduledTask {
            return ScheduledTask(
                ControllerAxisThreshold(
                    user,
                    condition.first,
                    condition.second,
                    Condition.Edge.RISING
                )
            )
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a falling-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun whenFalling(axis: Analog, threshold: (Float) -> Boolean): ScheduledTask {
            return ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.FALLING))
        }

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a falling-edge detection.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun whenFalling(condition: Pair<Analog, (Float) -> Boolean>): ScheduledTask {
            return ScheduledTask(
                ControllerAxisThreshold(
                    user,
                    condition.first,
                    condition.second,
                    Condition.Edge.FALLING
                )
            )
        }

        /**
         * Run a task when a controller button is held.
         * This condition will be evaluated continuously.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenHeld(button: Controls): ScheduledTask {
            return ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.ACTIVE))
        }

        /**
         * Run a task when a controller button is pressed (will run once when pressing the desired input).
         * This is the same as rising-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenPressed(button: Controls): ScheduledTask {
            return ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.RISING))
        }

        /**
         * Run a task when a controller button is released (will run once letting go of the desired input).
         * This is the same as falling-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenReleased(button: Controls): ScheduledTask {
            return ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.FALLING))
        }
    }

    /**
     * A task that will run when a condition is met.
     */
    inner class ScheduledTask(private val originalRunCondition: Condition) {
        /**
         * The ID (allocated task index) of the task that can be used to unbind and identify the binding.
         */
        @JvmField
        val id: Int

        /**
         * The task to run when the condition is met.
         */
        var taskToRun: Task = IdleTask()
            private set

        internal val runCondition: () -> Boolean
        internal var debouncing: Boolean = false
        internal var stopCondition: (() -> Boolean)? = null
        internal var muted = false

        private val and = ArrayList<BooleanSupplier>()
        private val or = ArrayList<BooleanSupplier>()

        init {
            // Run the task if the original expression is met,
            // and all AND conditions are met, or any OR conditions are met
            runCondition = {
                originalRunCondition.asBoolean
                        && and.stream().allMatch { obj -> obj.asBoolean }
                        || or.stream().anyMatch { obj -> obj.asBoolean }
            }
            allocatedTasks.add(this)
            id = allocatedTasks.size - 1
        }

        /**
         * Queue a task when the condition is met.
         * This task will run (and self-reset if finished) for as long as the condition is met.
         *
         * Note this means that the task provided will run from start-to-finish when the condition is true, which means
         * it *won't execute exclusively while the condition is met*, rather have the capability to be started when
         * the condition is met. This means continuous iterations of a true condition will try to keep this task queued
         * at all times, resetting the task internally when it is completed. Keep this in mind if working with
         * looping/long tasks, as you might experience runaway tasks.
         * See [finishIf] for fine-grain "run exclusively if" control.
         *
         * This method can only be called once per ScheduledTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        infix fun run(task: Task): ScheduledTask {
            if (taskToRun !is IdleTask) {
                throw EmergencyStop("A run(Task) method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.")
            }
            taskToRun = task
            return this
        }

        /**
         * Implicitly make a new [Lambda] to run as the condition is met.
         * This callback will requeue as many times as the trigger is met.
         *
         * This method can only be called once per ScheduledTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an [Lambda].
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        infix fun run(runnable: Runnable): ScheduledTask {
            return run(Lambda(runnable))
        }

        /**
         * Implicitly make a new [Lambda] to run as the condition is met.
         * This callback will requeue as many times as the trigger is met.
         *
         * This method can only be called once per ScheduledTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an [Lambda].
         *
         * @param name task name
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        fun run(name: String, runnable: Runnable): ScheduledTask {
            return run(Lambda(runnable).named(name))
        }

        /**
         * Queue a task when the condition is met, debouncing the task from queueing more than once the condition is met.
         *
         * This task will run, and a self-reset will not be propagated once the task is completed. Do note that this
         * effectively nullifies the trigger for the task, as it cannot auto-reset unless the task is manually reset
         * or designed to reset itself/run continuously. Managing the task passed here is up to the user.
         *
         * This method can only be called once per ScheduledTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        infix fun runOnce(task: Task): ScheduledTask {
            debouncing = true
            return run(task)
        }

        /**
         * Implicitly make a new [Lambda] to run once the condition is met, debouncing the task from queueing more than once the condition is met.
         *
         * This code block will run, and a self-reset will not be propagated once the task is completed. Do note that this
         * effectively nullifies the entire trigger for the task, as it cannot auto-reset. For a Runnable that can reset itself,
         * consider passing a [Lambda] to the [runOnce] method which will grant you access to the task's reset method.
         *
         * This method can only be called once per ScheduledTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an [Lambda].
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        infix fun runOnce(runnable: Runnable): ScheduledTask {
            return runOnce(Lambda(runnable))
        }

        /**
         * Implicitly make a new [Lambda] to run once the condition is met, debouncing the task from queueing more than once the condition is met.
         *
         * This code block will run, and a self-reset will not be propagated once the task is completed. Do note that this
         * effectively nullifies the entire trigger for the task, as it cannot auto-reset. For a Runnable that can reset itself,
         * consider passing a [Lambda] to the [runOnce] method which will grant you access to the task's reset method.
         *
         * This method can only be called once per ScheduledTask, see a TaskGroup for multiple task execution.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an [Lambda].
         *
         * @param name task name
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        fun runOnce(name: String, runnable: Runnable): ScheduledTask {
            return runOnce(Lambda(runnable).named(name))
        }

        /**
         * Mute this task from being a part of the Scheduler report.
         *
         * @return Current builder for additional task parameters
         */
        fun muted(): ScheduledTask {
            muted = true
            return this
        }

        /**
         * Chain an `AND` condition to the current conditional task.
         * Will be evaluated after the controller condition, and before the `OR` conditions.
         *
         * @param condition The AND condition to chain.
         * @return Current builder for additional task parameters
         */
        infix fun and(condition: BooleanSupplier): ScheduledTask {
            and.add(condition)
            return this
        }

        /**
         * Chain an `OR` condition to the current conditional task.
         * Will be evaluated after the controller and `AND` conditions.
         *
         * @param condition The OR condition to chain.
         * @return Current builder for additional task parameters
         */
        infix fun or(condition: BooleanSupplier): ScheduledTask {
            or.add(condition)
            return this
        }

        /**
         * Run a task assigned to in [run] in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * For Kotlin users, calling this method can be done with the notation `to`
         * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
         * or by calling the alias `after`.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        @SuppressLint("NoHardKeywords")
        infix fun `in`(interval: Measure<Time>): ScheduledTask {
            originalRunCondition.withActiveDelay(interval)
            return this
        }

        /**
         * Run a task assigned to in [run] in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        infix fun after(interval: Measure<Time>): ScheduledTask {
            return `in`(interval)
        }

        /**
         * Run the task assigned to in [run] until this condition is met. Once this condition is met, the task will
         * be forcefully stopped and the scheduler will move on. This is useful for continuous tasks.
         * If this method is called multiple times, an OR condition will be composed with the last condition.
         *
         * @param condition The condition to stop the task. Note the task will be auto-stopped if it finishes by itself,
         * this condition simply allows for an early finish if this condition is met.
         * @return Current builder for additional task parameters
         */
        infix fun finishIf(condition: BooleanSupplier): ScheduledTask {
            // Use prev to avoid a stack overflow
            val prev = stopCondition
            stopCondition = if (prev == null) {
                { condition.asBoolean }
            } else {
                { prev.invoke() || condition.asBoolean }
            }
            return this
        }

        override fun toString(): String {
            val out = Text.builder()
            out.append(if (taskToRun.dependency.isPresent) "Scheduling " else "Running ")
                .append("'")
                .append(taskToRun.toString())
                .append("'")
            val timeout = taskToRun.timeout to Seconds
            if (timeout * 1000.0 > Lambda.EPSILON_MS) {
                out.append(" (t=").append(timeout).append("s)")
            }
            if (taskToRun.isPriority) out.append(" (overriding)")
            if (originalRunCondition is ControllerButtonBind) {
                val handler = originalRunCondition
                out.append(" when GP")
                    .append(if (handler.controller.user == GamepadUser.ONE) 1 else 2)
                    .append("->")
                    .append(handler.button)
                    .append(" is ")
                    .append(handler.edge)
                val delay = originalRunCondition.getActiveDelay()
                if (delay.magnitude() > 0) {
                    out.append(" after ")
                        .append(originalRunCondition.getActiveDelay() to Seconds round 1)
                        .append("s")
                }
            } else {
                out.append(" when ")
                    .append(
                        originalRunCondition.toString().replace(BuildConfig.LIBRARY_PACKAGE_NAME + ".Scheduler", "")
                    )
                    .append(" is true")
            }
            out.append(if (and.isNotEmpty()) ", " + and.size + " extra AND condition(s)" else "")
                .append(if (or.isNotEmpty()) ", " + or.size + " extra OR condition(s)" else "")
                .append(if (debouncing) ", debouncing" else "")
                .append(if (muted) ", task status muted" else "")
            return out.toString()
        }
    }

    companion object {
        private var isMuted = false
    }
}
