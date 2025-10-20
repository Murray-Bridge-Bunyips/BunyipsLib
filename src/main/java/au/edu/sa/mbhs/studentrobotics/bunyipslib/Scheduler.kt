package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler.gamepad1
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler.gamepad2
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler.on
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler.update
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.isNear
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Comparison
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Condition
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.RepeatTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.util.cell.LateInitCell
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser
import java.util.Objects
import java.util.function.BiConsumer
import java.util.function.BooleanSupplier
import kotlin.math.abs

/**
 * Command-based paradigm scheduler and task plexus for BunyipsLib in TeleOp.
 * This is a singleton used to schedule and run tasks.
 *
 * Designed to mirror the WPILib command scheduler. Implementations must call [update] periodically.
 *
 * It is recommended to use the following static imports with the Scheduler:
 * ```java
 *  import static au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler.*;
 *  import static au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls.*;
 *  import static au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls.Analog.*;
 * ```
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
object Scheduler {
    private val subsystemsCell = LateInitCell<HashSet<BunyipsSubsystem>>()

    /**
     * Subsystems managed by the Scheduler.
     */
    @JvmStatic
    var subsystems by subsystemsCell
    private val triggerCache = HashMap<Int, Trigger>()
    private val gamepad1ButtonCache = HashMap<Controls, Trigger>()
    private val gamepad1AxisCache = HashMap<Int, Trigger>()
    private val gamepad2ButtonCache = HashMap<Controls, Trigger>()
    private val gamepad2AxisCache = HashMap<Int, Trigger>()
    private val activeTasks = LinkedHashSet<Task>()
    private val tasksToRemove = ArrayList<Task>()
    private val scheduledTasks = LinkedHashSet<ScheduledTask>()
    private var taskIdCount = 0
    private var disabled = false
    private var muted = false

    @JvmStatic
    @Hook(on = Hook.Target.PRE_START)
    private fun start() {
        if (!subsystemsCell.initialised)
            subsystems = BunyipsSubsystem.getInstances()
        val out = Text.builder()
        // Task count will account for tasks on subsystems that are not IdleTasks
        val taskCount =
            (scheduledTasks.size + subsystems.size - subsystems.stream().filter { it.isIdle() }.count()).toInt()
        val tasksWithNoDependencies = scheduledTasks.stream().filter { it.task.dependency.isEmpty }.count()
        out.append(
            "[Scheduler] active | Managing % subsystem(s) | % task(s) scheduled (% subsystem, % command)\n",
            subsystems.size,
            taskCount,
            taskCount - tasksWithNoDependencies,
            tasksWithNoDependencies
        )
        for (subsystem in subsystems) {
            out.append(" | %\n", subsystem.toVerboseString())
            for (binding in scheduledTasks) {
                val dep = binding.task.dependency
                if (dep.isEmpty || dep.get() != subsystem) continue
                out.append("    -> %\n", binding)
            }
        }
        for (binding in scheduledTasks) {
            if (binding.task.dependency.isPresent) continue
            out.append("  : %\n", binding)
        }
        Dbg.logd(out.toString())
        if (subsystems.isEmpty())
            throw RuntimeException("No BunyipsSubsystems were constructed!")
    }

    @JvmStatic
    @Hook(on = Hook.Target.POST_STOP)
    private fun cleanup() {
        subsystemsCell.invalidate()
        triggerCache.clear()
        gamepad1ButtonCache.clear()
        gamepad1AxisCache.clear()
        gamepad2ButtonCache.clear()
        gamepad2AxisCache.clear()
        activeTasks.clear()
        tasksToRemove.clear()
        scheduledTasks.clear()
        disabled = false
        muted = false
        taskIdCount = 0
    }

    /**
     * Use a specific subset of [subsystems] for the Scheduler.
     *
     * By default, all constructed [BunyipsSubsystem] instances will be auto-detected and used.
     */
    @JvmStatic
    fun use(vararg subsystems: BunyipsSubsystem) {
        this.subsystems = subsystems.toHashSet()
    }

    /**
     * Schedules a task to execute.
     *
     * **Note:** Using this method directly is not recommended,
     * you should create a binding with [on], [gamepad1], or [gamepad2].
     *
     * This method can be manually called for advanced operations such as detaching or forking tasks. Do note this
     * method is not thread-safe.
     */
    @JvmStatic
    fun schedule(task: Task) {
        // Do nothing if we're already running that task
        if (activeTasks.contains(task))
            return
        // Important step that we ensure the task is ready for execution
        task.finishNow()
        task.reset()
        activeTasks.add(task)
    }

    /**
     * Updates subsystems, runs bindings, and schedules and executes tasks.
     */
    @JvmStatic
    fun update() {
        if (disabled) return

        if (!muted) {
            val tasksWithDependencies = scheduledTasks.stream().filter { it.task.dependency.isPresent }.count()
            // Task count will account for tasks on subsystems that are not IdleTasks, and also subsystem tasks
            val taskCount = scheduledTasks.size - tasksWithDependencies + subsystems.size -
                    subsystems.stream().filter { it.isIdle }.count()
            DualTelemetry.smartAdd(
                "\nRunning % task% (%s, %c) on % subsystem%",
                taskCount,
                if (taskCount == 1L) "" else "s",
                tasksWithDependencies + taskCount - scheduledTasks.size,
                scheduledTasks.size - tasksWithDependencies,
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
            for (binding in scheduledTasks) {
                if (binding.task.dependency.isPresent // Whether the task is never run from the Scheduler (and task reports were handled above)
                    || !binding.task.isRunning // Whether this task is actually running
                    || binding.muted // Whether the task has declared itself as muted
                ) {
                    continue
                }
                val deltaTime = binding.task.deltaTime to Seconds round 1
                DualTelemetry.smartAdd(
                    "<small><b>Scheduler</b> (c.) <font color='gray'>|</font> <b>%</b> -> %</small>",
                    binding.task,
                    if (deltaTime == 0.0) "active" else deltaTime.toString() + "s"
                )
            }
        }

        // 1. Update all subsystems
        subsystems.forEach { it.update() }

        // 2. Poll all binds to populate activeTasks with new tasks
        scheduledTasks.forEach { it.poll() }

        // 3. Run scheduled tasks
        for (task in activeTasks) {
            if (task.isFinished) {
                // We're done here, schedule for removal
                tasksToRemove.add(task)
                continue
            }
            task.execute()
            if (task.dependency.isEmpty) {
                // Update finish conditions for non-subsystem tasks as it is not done elsewhere
                task.poll()
            }
        }

        // 4. Cleanup finished tasks
        activeTasks.removeAll(tasksToRemove)
        tasksToRemove.clear()
    }

    /**
     * Disables ability to call [update].
     */
    @JvmStatic
    fun disable() {
        disabled = true
    }

    /**
     * Resumes ability to call [update].
     */
    @JvmStatic
    fun enable() {
        disabled = false
    }

    /**
     * Unbinds a scheduled task by the [id].
     *
     * The [id] is equivalent to the zero-indexed number of constructed binds from initialisation.
     */
    @JvmStatic
    fun unbind(id: Int) {
        scheduledTasks.removeIf { it.id == id }
    }

    /**
     * Mutes Scheduler telemetry.
     */
    @JvmStatic
    fun mute() {
        muted = true
    }

    /**
     * Unmutes Scheduler telemetry.
     */
    @JvmStatic
    fun unmute() {
        muted = false
    }

    /**
     * Creates a [Trigger] for the boolean [condition].
     *
     * Attempts to use an internal cache for repeated triggers with the same hash code.
     */
    @JvmStatic
    infix fun on(condition: BooleanSupplier) = triggerCache.computeIfAbsent(condition.hashCode()) {
        // Attempt to hit our cache but this may fail if new anonymous functions are used which is common
        Trigger(condition)
    }

    /**
     * Creates a button or axis trigger binder to create a [Trigger] for `gamepad1`.
     */
    @JvmStatic
    fun gamepad1() = GamepadTrigger(GamepadUser.ONE, BunyipsLib.opMode.gamepad1)

    /**
     * Creates a button or axis trigger binder to create a [Trigger] for `gamepad2`.
     */
    @JvmStatic
    fun gamepad2() = GamepadTrigger(GamepadUser.TWO, BunyipsLib.opMode.gamepad2)

    /**
     * Button or axis binding creator.
     *
     * Utilises internal caches for global controller-based triggers.
     */
    class GamepadTrigger(
        private val user: GamepadUser,
        @JvmField
        val instance: Gamepad
    ) {
        /**
         * Creates a [Trigger] for when this [button] is pressed.
         */
        infix fun button(button: Controls): Trigger {
            val cache = if (user == GamepadUser.ONE) gamepad1ButtonCache else gamepad2ButtonCache
            return cache.computeIfAbsent(button) { Trigger(ButtonBind(instance, button)) }
        }

        /**
         * Creates a [Trigger] for when this [axis] value is less than the [threshold].
         */
        fun axisLessThan(axis: Controls.Analog, threshold: Double): Trigger {
            val cache = if (user == GamepadUser.ONE) gamepad1AxisCache else gamepad2AxisCache
            return cache.computeIfAbsent(Objects.hash(axis, threshold, Comparison.LESS_THAN)) {
                Trigger(AxisBind(instance, axis, threshold, Comparison.LESS_THAN))
            }
        }

        /**
         * Creates a [Trigger] for when this [axis] value is greater than the [threshold].
         */
        fun axisGreaterThan(axis: Controls.Analog, threshold: Double): Trigger {
            val cache = if (user == GamepadUser.ONE) gamepad1AxisCache else gamepad2AxisCache
            return cache.computeIfAbsent(Objects.hash(axis, threshold, Comparison.GREATER_THAN)) {
                Trigger(AxisBind(instance, axis, threshold, Comparison.GREATER_THAN))
            }
        }

        /**
         * Creates a [Trigger] for when this [axis]'s magnitude (non-signed value) is greater than the [threshold].
         */
        fun axisMagnitudeGreaterThan(axis: Controls.Analog, threshold: Double): Trigger {
            val cache = if (user == GamepadUser.ONE) gamepad1AxisCache else gamepad2AxisCache
            return cache.computeIfAbsent(Objects.hash(axis, threshold, Comparison.MAGNITUDE_GREATER_THAN)) {
                Trigger(AxisBind(instance, axis, threshold, Comparison.MAGNITUDE_GREATER_THAN))
            }
        }

        data class ButtonBind(val gamepad: Gamepad, val button: Controls) : BooleanSupplier {
            override fun toString() = "gamepad${Controller.tryGetUser(gamepad)?.id ?: "?"} button $button"
            override fun getAsBoolean() = Controls.isSelected(gamepad, button)
        }

        data class AxisBind(
            val gamepad: Gamepad,
            val axis: Controls.Analog,
            val threshold: Double,
            val comparison: Comparison
        ) : BooleanSupplier {
            override fun toString() =
                "gamepad${Controller.tryGetUser(gamepad)?.id ?: "?"} axis $axis ${comparison.symbol} $threshold"

            override fun getAsBoolean() = when (comparison) {
                Comparison.LESS_THAN -> Controls.Analog.get(gamepad, axis) < threshold
                Comparison.GREATER_THAN -> Controls.Analog.get(gamepad, axis) > threshold
                Comparison.MAGNITUDE_GREATER_THAN -> abs(Controls.Analog.get(gamepad, axis)) > threshold
                Comparison.LESS_THAN_OR_EQUAL -> Controls.Analog.get(gamepad, axis) <= threshold
                Comparison.GREATER_THAN_OR_EQUAL -> Controls.Analog.get(gamepad, axis) >= threshold
                Comparison.EQUAL -> Controls.Analog.get(gamepad, axis).isNear(threshold, 1e-6)
                Comparison.NOT_EQUAL -> !Controls.Analog.get(gamepad, axis).isNear(threshold, 1e-6)
            }
        }
    }


    /**
     * A [condition] that will link to and execute some [Task].
     */
    class Trigger(val condition: BooleanSupplier) : Condition<Trigger>(condition) {
        private var last: ScheduledTask? = null

        private inline fun buildScheduledTask(type: String, block: () -> ScheduledTask) = apply {
            val scheduledTask = block.invoke()
            last = scheduledTask
            Dbg.logv(javaClass, "binding $condition $type -> ${scheduledTask.task}")
        }

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`, or `true` to `false`.
         */
        infix fun onChange(task: Task) = buildScheduledTask("onChange") {
            ScheduledTask(task, this) { prev, curr ->
                if (prev != curr)
                    schedule(task)
            }
        }

        infix fun onChange(runnable: Runnable) = onChange(null, runnable)
        fun onChange(name: String?, runnable: Runnable) =
            onChange(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`.
         */
        infix fun onTrue(task: Task) = buildScheduledTask("onTrue") {
            ScheduledTask(task, this) { prev, curr ->
                if (!prev && curr)
                    schedule(task)
            }
        }

        infix fun onTrue(runnable: Runnable) = onTrue(null, runnable)
        fun onTrue(name: String?, runnable: Runnable) =
            onTrue(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`.
         */
        infix fun onFalse(task: Task) = buildScheduledTask("onFalse") {
            ScheduledTask(task, this) { prev, curr ->
                if (prev && !curr)
                    schedule(task)
            }
        }

        infix fun onFalse(runnable: Runnable) = onFalse(null, runnable)
        fun onFalse(name: String?, runnable: Runnable) =
            onFalse(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`,
         * and finishes the task when the same condition changes back to `false`.
         *
         * Does not restart the task if it finishes by itself while the condition is still `true`.
         * Use a [RepeatTask] for this behaviour.
         */
        infix fun whileTrue(task: Task) = buildScheduledTask("whileTrue") {
            ScheduledTask(task, this) { prev, curr ->
                if (!prev && curr)
                    schedule(task)
                else if (prev && !curr)
                    task.finish()
            }
        }

        infix fun whileTrue(runnable: Runnable) = whileTrue(null, runnable)
        fun whileTrue(name: String?, runnable: Runnable) =
            whileTrue(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`,
         * and finishes the task when the same condition changes back to `true`.
         *
         * Does not restart the task if it finishes by itself while the condition is still `false`.
         * Use a [RepeatTask] for this behaviour.
         */
        infix fun whileFalse(task: Task) = buildScheduledTask("whileFalse") {
            ScheduledTask(task, this) { prev, curr ->
                if (prev && !curr)
                    schedule(task)
                else if (!prev && curr)
                    task.finish()
            }
        }

        infix fun whileFalse(runnable: Runnable) = whileFalse(null, runnable)
        fun whileFalse(name: String?, runnable: Runnable) =
            whileFalse(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`,
         * and finishes the task under the same condition (`false` to `true`) if the task is still running.
         */
        infix fun toggleOnTrue(task: Task) = buildScheduledTask("toggleOnTrue") {
            ScheduledTask(task, this) { prev, curr ->
                if (!prev && curr) {
                    if (task.isRunning)
                        task.finish()
                    else
                        schedule(task)
                }
            }
        }

        infix fun toggleOnTrue(runnable: Runnable) = toggleOnTrue(null, runnable)
        fun toggleOnTrue(name: String?, runnable: Runnable) =
            toggleOnTrue(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`,
         * and finishes the task under the same condition (`true` to `false`) if the task is still running.
         */
        infix fun toggleOnFalse(task: Task) = buildScheduledTask("toggleOnFalse") {
            ScheduledTask(task, this) { prev, curr ->
                if (prev && !curr) {
                    if (task.isRunning)
                        task.finish()
                    else
                        schedule(task)
                }
            }
        }

        infix fun toggleOnFalse(runnable: Runnable) = toggleOnFalse(null, runnable)
        fun toggleOnFalse(name: String?, runnable: Runnable) =
            toggleOnFalse(Lambda { runnable.run() }.also { name?.let { n -> it.named(n) } })

        /**
         * Descends into the last binded [ScheduledTask] for this trigger.
         *
         * Useful for accessing binding-specific information to stop building more [ScheduledTask] instances.
         */
        fun descend(): ScheduledTask {
            return last
                ?: throw Exceptions.EmergencyStop("Cannot descend into a ScheduledTask as none have been bound for this specific Trigger.")
        }

        override fun newInstance(edge: Edge, supplier: BooleanSupplier): Trigger {
            return Trigger(supplier).also {
                it.edge = edge
                it.last = last
            }
        }

        /**
         * Alias for [not].
         */
        fun negate() = not()

        /**
         * Alias for [and].
         */
        operator fun plus(other: Trigger) = and(other)

        /**
         * Alias for [or].
         */
        operator fun div(other: Trigger) = or(other)
    }

    /**
     * A bound [task] that will schedule on the given [binding], accepting the previous state
     * of the trigger and current state of the trigger.
     */
    class ScheduledTask(
        @JvmField
        val task: Task,
        @JvmField
        val trigger: Trigger,
        // (prev, curr)
        @JvmField
        val binding: BiConsumer<Boolean, Boolean>
    ) {
        private var prev = trigger.asBoolean
        internal var muted = false

        init {
            scheduledTasks.add(this)
        }

        /**
         * Sequential [id] for this scheduled task that can be used to unbind this task.
         */
        @JvmField
        val id = taskIdCount++

        /**
         * Mute telemetry for this scheduled task.
         */
        fun mute() = apply {
            muted = true
        }

        /**
         * Unmute telemetry for this scheduled task.
         */
        fun unmute() = apply {
            muted = false
        }

        /**
         * Polls the [trigger] and executes the [binding] connected to this particular [task].
         *
         * Automatically called by [update].
         */
        fun poll() {
            val curr = trigger.asBoolean
            binding.accept(prev, curr)
            prev = curr
        }

        override fun toString(): String {
            val out = Text.builder()
            out.append(if (task.dependency.isPresent) "Scheduling " else "Running ")
                .append("'")
                .append(task.toString())
                .append("'")
            val timeout = task.timeout to Seconds
            if (timeout * 1000.0 > Lambda.EPSILON_MS) {
                out.append(" (t=").append(timeout).append("s)")
            }
            if (task.isPriority) out.append(" (overriding)")
            out.append("on ").append(trigger)
            out.append(if (muted) ", task status muted" else "")
            return out.toString()
        }
    }
}