package au.edu.sa.mbhs.studentrobotics.bunyipslib

import android.annotation.SuppressLint
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Condition
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls.Analog
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls.Companion.get
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.qualcomm.robotcore.hardware.Gamepad
import java.util.function.BooleanSupplier

/**
 * Scheduler and command plexus for use with the BunyipsLib task system in TeleOp.
 *
 * @author Lucas Bubner, 2024
 * @see CommandBasedBunyipsOpMode
 * @since 1.0.0-pre
 */
class Scheduler {
    private val subsystems = ArrayList<BunyipsSubsystem>()
    private val allocatedTasks = ArrayList<ScheduledTask>()

    /**
     * Get all allocated tasks.
     */
    fun getAllocatedTasks() = allocatedTasks.toTypedArray<ScheduledTask>()

    /**
     * All subsystems attached to and managed by the Scheduler.
     */
    val managedSubsystems: Array<BunyipsSubsystem>
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
        if (subsystems.isEmpty())
            Dbg.warn(javaClass, "Caution: No subsystems were added for the Scheduler to update.")
        else
            Dbg.logv(javaClass, "Added % subsystem(s) to update.", dispatch.size)
    }

    /**
     * Disable all subsystems attached to the Scheduler.
     */
    fun disable() {
        subsystems.forEach { it.disable() }
    }

    /**
     * Enable all subsystems attached to the Scheduler, unless they failed from null assertion.
     */
    fun enable() {
        subsystems.forEach { it.enable() }
    }

    /**
     * Run the scheduler. This will run all subsystems and tasks allocated to the scheduler.
     * This should be called in the `activeLoop()` method of the [BunyipsOpMode], and is automatically called
     * in [CommandBasedBunyipsOpMode].
     */
    fun run() {
        subsystems.forEach { it.update() }

        if (!isMuted) {
            // Task count will account for tasks on subsystems that are not IdleTasks, and also subsystem tasks
            val taskCount = (allocatedTasks.size - allocatedTasks.stream()
                .filter { task -> task.taskToRun.dependency.isPresent }.count()
                    + subsystems.size - subsystems.stream().filter { s -> s.isIdle }.count())
            DualTelemetry.smartAdd(
                "\nRunning % task% (%s, %c) on % subsystem%",
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
                DualTelemetry.smartAdd(format = report)
            }
            for (task in allocatedTasks) {
                if (task.taskToRun.dependency.isPresent // Whether the task is never run from the Scheduler (and task reports were handled above)
                    || !task.taskToRun.isRunning // Whether this task is actually running
                    || task.muted // Whether the task has declared itself as muted
                ) {
                    continue
                }
                val deltaTime = task.taskToRun.deltaTime to Seconds round 1
                DualTelemetry.smartAdd(
                    "<small><b>Scheduler</b> (c.) <font color='gray'>|</font> <b>%</b> -> %</small>",
                    task.taskToRun,
                    if (deltaTime == 0.0) "active" else deltaTime.toString() + "s"
                )
            }
        }

        for (task in allocatedTasks) {
            val condition = task.runCondition.invoke()
            if (task.stopCondition == null) task.stopCondition = { false }
            if (condition || task.taskToRun.isRunning) {
                // Terminate current task if we hit the stop condition, or early return if we've already finished
                if (task.stopCondition!!.invoke() || task.taskToRun.isFinished) {
                    // No-ops if already stopped
                    task.taskToRun.finishNow()
                    continue
                }
                task.taskToRun.execute()
                if (task.taskToRun.dependency.isEmpty) {
                    // Update finish conditions for non-subsystem tasks as it is not done elsewhere
                    task.taskToRun.poll()
                }
            } else if (task.taskToRun.isFinished && !task.debouncing) {
                // Smart button toggles will need to reset the debounce to function if the task is finished early
                task.useSmartRetrigger?.let { (c, b) -> c.resetDebounce(b) }
                // For tasks that are not debouncing and their conditions are met again, we restart them
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
     * @since 7.0.0
     */
    fun unbind(index: Int) {
        Dbg.logd(javaClass, "unbound task %: %", allocatedTasks.size - 1, allocatedTasks.removeAt(index))
    }

    /**
     * Unbind a scheduled task from the scheduler.
     *
     * @param task The [ScheduledTask] to unbind.
     * @since 7.0.0
     */
    fun unbind(task: ScheduledTask) {
        if (allocatedTasks.remove(task))
            Dbg.logd(javaClass, "unbound task %: %", allocatedTasks.size, task)
    }

    /**
     * Create a new controller trigger creator.
     *
     * For Kotlin users, calling this method can be done with the notation &#96;when&#96;
     * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
     * or by calling the alias `on`.
     *
     * @param user The Controller instance to use.
     * @return The controller trigger creator.
     */
    @SuppressLint("NoHardKeywords")
    fun `when`(user: Gamepad) = ControllerTriggerCreator(user)

    /**
     * Create a new controller trigger creator.
     *
     * @param user The Controller instance to use.
     * @return The controller button trigger creator.
     */
    fun on(user: Gamepad) = ControllerTriggerCreator(user)

    /**
     * Create a new controller trigger creator for the driver (gamepad 1).
     *
     * @return The controller trigger creator.
     */
    fun driver() = ControllerTriggerCreator(BunyipsLib.opMode.gamepad1)

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
    fun operator() = ControllerTriggerCreator(BunyipsLib.opMode.gamepad2)

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
     * For Kotlin users, calling this method can be done with the notation &#96;when&#96;
     * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
     * or by calling the alias `on`.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @SuppressLint("NoHardKeywords")
    fun `when`(condition: BooleanSupplier) =
        if (condition is Condition) {
            ScheduledTask(condition)
        } else {
            ScheduledTask(Condition(condition))
        }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun on(condition: BooleanSupplier) = `when`(condition)

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a rising-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun whenRising(condition: BooleanSupplier) = ScheduledTask(
        Condition(
            Condition.Edge.RISING,
            condition
        )
    )

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a falling-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    fun whenFalling(condition: BooleanSupplier) = ScheduledTask(
        Condition(
            Condition.Edge.FALLING,
            condition
        )
    )

    /**
     * Run a task immediately with no start condition. This is the same as calling `.when(() -> true)`.
     *
     * @return Task scheduling builder
     */
    fun immediately() = ScheduledTask(Condition { true })

    private class ControllerButtonBind(
        val controller: Gamepad,
        val button: Controls,
        edge: Edge
    ) : Condition(edge, { controller[button] }) {
        override fun toString() = "Button($edge):GP${Controller.tryGetUser(controller)?.id ?: "?"}->$button"
    }

    private class ControllerAxisThreshold(
        private val controller: Gamepad,
        private val axis: Analog,
        threshold: (Float) -> Boolean,
        edge: Edge
    ) : Condition(edge, { threshold.invoke(controller[axis]) }) {
        override fun toString() = "Axis($edge):GP${Controller.tryGetUser(controller)?.id ?: "?"}->$axis"
    }

    /**
     * Controller trigger creator.
     */
    inner class ControllerTriggerCreator(private val user: Gamepad) {
        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * For Kotlin users, calling this method can be done with the notation &#96;when&#96;
         * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
         * or by calling the alias `on`.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        @SuppressLint("NoHardKeywords")
        fun `when`(axis: Analog, threshold: (Float) -> Boolean) =
            ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.ACTIVE))

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun on(axis: Analog, threshold: (Float) -> Boolean) = `when`(axis, threshold)

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated continuously.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun on(condition: Pair<Analog, (Float) -> Boolean>) = `when`(condition.first, condition.second)

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a rising-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun whenRising(axis: Analog, threshold: (Float) -> Boolean) =
            ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.RISING))

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a rising-edge detection.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun whenRising(condition: Pair<Analog, (Float) -> Boolean>) =
            ScheduledTask(ControllerAxisThreshold(user, condition.first, condition.second, Condition.Edge.RISING))

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a falling-edge detection.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Task scheduling builder
         */
        fun whenFalling(axis: Analog, threshold: (Float) -> Boolean) =
            ScheduledTask(ControllerAxisThreshold(user, axis, threshold, Condition.Edge.FALLING))

        /**
         * Run a task once this analog axis condition is met.
         * This condition will be evaluated according to a falling-edge detection.
         *
         * @param condition The condition to meet.
         * @return Task scheduling builder
         */
        infix fun whenFalling(condition: Pair<Analog, (Float) -> Boolean>) =
            ScheduledTask(ControllerAxisThreshold(user, condition.first, condition.second, Condition.Edge.FALLING))

        /**
         * Run a task when a controller button is held.
         * This condition will be evaluated continuously.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenHeld(button: Controls) =
            ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.ACTIVE))

        /**
         * Run a task when a controller button is pressed (will run once when pressing the desired input).
         * This is the same as rising-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenPressed(button: Controls) =
            ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.RISING))

        /**
         * Run a task when a controller button is released (will run once letting go of the desired input).
         * This is the same as falling-edge detection.
         *
         * @param button The button of the controller.
         * @return Task scheduling builder
         */
        infix fun whenReleased(button: Controls) =
            ScheduledTask(ControllerButtonBind(user, button, Condition.Edge.FALLING))
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
        internal var useSmartRetrigger: Pair<Controller, Controls>? = null
        internal var muted: Boolean = false

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
            Dbg.logv(javaClass, "allocating task binding % for % ...", id, originalRunCondition)
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
        infix fun run(task: Task) = apply {
            if (taskToRun !is IdleTask) {
                throw Exceptions.EmergencyStop("A run(Task) method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.")
            }
            taskToRun = task
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
        infix fun run(runnable: Runnable) = run(Lambda(runnable))

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
        fun run(name: String, runnable: Runnable) = run(Lambda(runnable).named(name))

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
        infix fun runOnce(task: Task) = run(task).also { debouncing = true }

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
        infix fun runOnce(runnable: Runnable) = runOnce(Lambda(runnable))

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
        fun runOnce(name: String, runnable: Runnable) = runOnce(Lambda(runnable).named(name))

        /**
         * Mute this task from being a part of the Scheduler report.
         *
         * @return Current builder for additional task parameters
         */
        fun muted() = apply {
            muted = true
        }

        /**
         * Chain an `AND` condition to the current conditional task.
         * Will be evaluated after the controller condition, and before the `OR` conditions.
         *
         * @param condition The AND condition to chain.
         * @return Current builder for additional task parameters
         */
        infix fun and(condition: BooleanSupplier) = apply {
            and.add(condition)
        }

        /**
         * Chain an `OR` condition to the current conditional task.
         * Will be evaluated after the controller and `AND` conditions.
         *
         * @param condition The OR condition to chain.
         * @return Current builder for additional task parameters
         */
        infix fun or(condition: BooleanSupplier) = apply {
            or.add(condition)
        }

        /**
         * Run a task assigned to in [run] in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * For Kotlin users, calling this method can be done with the notation &#96;to&#96;
         * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
         * or by calling the alias `after`.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        @SuppressLint("NoHardKeywords")
        infix fun `in`(interval: Measure<Time>) = apply {
            originalRunCondition.withActiveDelay(interval)
        }

        /**
         * Run a task assigned to in [run] in a certain amount of time of the condition remaining true.
         * This will delay the activation of the task by the specified amount of time of the condition remaining true.
         * If this method is called multiple times, the last time directive will be used.
         *
         * @param interval The time interval
         * @return Current builder for additional task parameters
         */
        infix fun after(interval: Measure<Time>) = `in`(interval)

        /**
         * Run the task assigned to in [run] until this condition is met. Once this condition is met, the task will
         * be forcefully stopped and the scheduler will move on. This is useful for continuous tasks.
         * If this method is called multiple times, an OR condition will be composed with the last condition.
         *
         * @param condition The condition to stop the task. Note the task will be auto-stopped if it finishes by itself,
         * this condition simply allows for an early finish if this condition is met.
         * @return Current builder for additional task parameters
         */
        infix fun finishIf(condition: BooleanSupplier) = apply {
            // Use prev to avoid a stack overflow
            val prev = stopCondition
            stopCondition = if (prev == null) {
                { condition.asBoolean }
            } else {
                { prev.invoke() || condition.asBoolean }
            }
        }

        /**
         * Automagic finish condition that will run the task assigned in [run] until the button used to trigger
         * this execution is retriggered. This is effectively a [finishIf] convenience method for an edge detector applied
         * to the button you used to declare this task binding, creating a task toggle.
         *
         * This method is equivalent to writing:
         * ```java
         * gp1().whenPressed(Controls.A)
         *      .run(/* ... */)
         *     .finishIf(() -> gamepad1.getDebounced(Controls.A)) // Common pattern to implement toggles
         * ```
         *
         * However, this method will also take efforts to manage the [Controller] debounce resets (through `resetDebounce`)
         * in the event your task finishes, which cannot be replicated through a simple [finishIf]. Without proper debounce resets,
         * your task may require two triggers to restart if it is stopped early or automatically, since `getDebounced`
         * is a stateful function unaware of your task's running state by default. This method also reduces repetition
         * that can introduce typo bugs.
         *
         * **WARNING:** This method will only work if you have used a [ControllerButtonBind] (such as through `whenPressed`, etc.)
         * AND you used a [Controller] instance to create the bind (such as by a [BunyipsOpMode]).
         *
         * @since 7.3.0
         */
        fun finishIfButtonRetriggered() = apply {
            if (originalRunCondition !is ControllerButtonBind)
                throw Exceptions.EmergencyStop("finishIfButtonRetriggered() called when bind was not created through a ControllerButtonBind.")
            if (originalRunCondition.controller !is Controller)
                throw Exceptions.EmergencyStop("finishIfButtonRetriggered() called when underlying controller used for bind is not a Controller instance.")
            useSmartRetrigger = originalRunCondition.controller to originalRunCondition.button
            finishIf { originalRunCondition.controller.getDebounced(originalRunCondition.button) }
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
                    .append(Controller.tryGetUser(handler.controller)?.id ?: "?")
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

    /**
     * Mute all Scheduler instances telemetry for the rest of the OpMode.
     */
    fun mute() {
        isMuted = true
    }

    /**
     * Unmute all Scheduler instances telemetry for the rest of the OpMode.
     */
    fun unmute() {
        isMuted = false
    }

    companion object {
        private var isMuted = false

        /**
         * Unmute all Scheduler instances telemetry for the rest of the OpMode.
         */
        @JvmStatic
        @JvmName("muteTelemetry")
        fun mute() {
            isMuted = true
        }

        /**
         * Unmute all Scheduler instances telemetry for the rest of the OpMode.
         */
        @JvmStatic
        @JvmName("unmuteTelemetry")
        fun unmute() {
            isMuted = false
        }

        @JvmStatic
        @Hook(on = Hook.Target.POST_STOP)
        private fun reset() {
            isMuted = false
        }
    }
}
