package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Condition
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.RepeatTask
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.util.cell.LateInitCell
import java.util.function.BiConsumer
import java.util.function.BooleanSupplier

/**
 * Command-based paradigm scheduler and task plexus for BunyipsLib in TeleOp.
 *
 * Designed to mirror the WPILib command scheduler.
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
    private val scheduledTasks = ArrayList<ScheduledTask>()
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
     * This method can be manually called for advanced operations such as detaching or forking tasks.
     */
    @JvmStatic
    fun schedule(task: Task) {

    }

    /**
     * Updates subsystems, runs bindings, and schedules and executes tasks.
     */
    @JvmStatic
    fun update() {
        if (disabled) return

        if (!muted) {
            // TODO: test these two counters
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

        // TODO
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
     */
    infix fun on(condition: BooleanSupplier) = Trigger(condition)

    /**
     * Creates a button or axis trigger binder to create a [Trigger] for `gamepad1`.
     */
    fun gamepad1() = GamepadTrigger(BunyipsLib.opMode.gamepad1)

    /**
     * Creates a button or axis trigger binder to create a [Trigger] for `gamepad2`.
     */
    fun gamepad2() = GamepadTrigger(BunyipsLib.opMode.gamepad2)

    /**
     * Button or axis binding creator.
     */
    class GamepadTrigger(val gamepad: Gamepad) {
        // TODO
        //  fun button(button: Controls)
        //  fun axis(...)
    }

    /**
     * A [condition] that will link to and execute some [Task].
     */
    class Trigger(val condition: BooleanSupplier): Condition<Trigger>(condition) {
        private var last: ScheduledTask? = null

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`, or `true` to `false`.
         */
        infix fun onChange(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (prev != curr)
                    schedule(task)
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`.
         */
        infix fun onTrue(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (!prev && curr)
                    schedule(task)
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`.
         */
        infix fun onFalse(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (prev && !curr)
                    schedule(task)
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`,
         * and finishes the task when the same condition changes back to `false`.
         *
         * Does not restart the task if it finishes by itself while the condition is still `true`.
         * Use a [RepeatTask] for this behaviour.
         */
        infix fun whileTrue(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (!prev && curr)
                    schedule(task)
                else if (prev && !curr)
                    task.finish()
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`,
         * and finishes the task when the same condition changes back to `true`.
         *
         * Does not restart the task if it finishes by itself while the condition is still `false`.
         * Use a [RepeatTask] for this behaviour.
         */
        infix fun whileFalse(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (prev && !curr)
                    schedule(task)
                else if (!prev && curr)
                    task.finish()
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `false` to `true`,
         * and finishes the task under the same condition (`false` to `true`) if the task is running.
         */
        infix fun toggleOnTrue(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (!prev && curr) {
                    if (task.isRunning) {
                        task.finish()
                    } else {
                        schedule(task)
                    }
                }
            }})
        }

        /**
         * Schedules [task] when the [condition] changes from `true` to `false`,
         * and finishes the task under the same condition (`true` to `false`) if the task is running.
         */
        infix fun toggleOnFalse(task: Task) = apply {
            scheduledTasks.add(ScheduledTask(task) { prev, curr -> {
                if (prev && !curr) {
                    if (task.isRunning) {
                        task.finish()
                    } else {
                        schedule(task)
                    }
                }
            }})
        }

        /**
         * Descends into the last binded [ScheduledTask] for this trigger.
         *
         * Useful for accessing binding-specific information to stop building more [ScheduledTask] instances.
         */
        fun descend(): ScheduledTask {
            return last ?: throw Exceptions.EmergencyStop("Cannot descend into a ScheduledTask as none have been bound for this specific Trigger.")
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
     * A binded [task] that will schedule on the given [binding], accepting the previous state
     * of the trigger and current state of the trigger.
     */
    // (prev, curr)
    data class ScheduledTask(val task: Task, val binding: BiConsumer<Boolean, Boolean>) {
        internal var muted = false

        /**
         * Sequential [id] for this scheduled task that can be used to unbind this task.
         */
        @JvmField
        val id = taskIdCount++

        /**
         * Mute telemetry for this scheduled task.
         */
        fun mute() {
            muted = true
        }

        /**
         * Unmute telemetry for this scheduled task.
         */
        fun unmute() {
            muted = false
        }
    }
}