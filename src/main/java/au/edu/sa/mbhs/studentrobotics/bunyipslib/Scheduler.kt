package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import dev.frozenmilk.util.cell.LateInitCell
import java.util.function.BiFunction
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
    @JvmStatic
    var subsystems by subsystemsCell
    private val scheduledTasks = ArrayList<ScheduledTask>()
    private var taskIdCount = 0
    private var disabled = false

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
            for (task in scheduledTasks) {
                val dep = task.task.dependency
                if (dep.isEmpty || dep.get() != subsystem) continue
                out.append("    -> %\n", task)
            }
        }
        for (task in scheduledTasks) {
            if (task.task.dependency.isPresent) continue
            out.append("  : %\n", task)
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

    @JvmStatic
    fun use(vararg subsystems: BunyipsSubsystem) {
        this.subsystems = subsystems.toHashSet()
    }

    @JvmStatic
    fun update() {
        if (disabled) return
        // TODO
    }

    @JvmStatic
    fun disable() {
        disabled = true
    }

    @JvmStatic
    fun enable() {
        disabled = false
    }

    @JvmStatic
    fun unbind(index: Int) {
        // TODO
    }

    // TODO
    //  on(BooleanSupplier) to transform into a Trigger
    //  gamepad1() [gamepad button() and axis controls from previous classes to transform into Trigger]
    //  gamepad2()
    //  docs for public things

    class Trigger(val condition: BooleanSupplier) {
        // TODO
        //  onChange(Task) [trigger implementation extends Condition]
        //  onTrue(Task)   [when one of these methods are called, add ScheduledTask based on current state]
        //  onFalse(Task)
        //  whileTrue(Task)
        //  whileFalse(Task)
        //  toggleOnTrue(Task)
        //  toggleOnFalse(Task)
        //  and(Trigger)
        //  or(Trigger)
        //  negate()/not()
    }

    // (prev, curr) -> run
    data class ScheduledTask(val task: Task, val condition: BiFunction<Boolean, Boolean, Boolean>) {
        // TODO: with the trigger system assigning an ID here might be inaccessible
        val id = taskIdCount++
    }
}