package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import java.util.function.BooleanSupplier

/**
 * Builder pattern implementation for [Task] instances.
 * 
 * @author Lucas Bubner, 2024
 * @since 6.1.0
 */
class TaskGen {
    private var name = "Task"
    private var timeout = Task.INFINITE_TIMEOUT
    private var subsystem: BunyipsSubsystem? = null
    private var overriding = false
    private var muted = false
    private var init = {}
    private var loop = {}
    private var until = { false }
    private var finish = {}
    private var interrupt = {}
    private var reset = {}

    /**
     * Sets the [name] of the task.
     */
    infix fun name(name: String) = apply { this.name = name }
    /**
     * Sets the [timeout] of the task.
     */
    infix fun timeout(taskTimeout: Measure<Time>) = apply { timeout = taskTimeout }
    /**
     * Sets the [subsystem] to be used by the task.
     */
    infix fun on(onSubsystem: BunyipsSubsystem) = apply { subsystem = onSubsystem }
    /**
     * Sets whether the task is [overriding] the current [subsystem] task if one exists.
     */
    infix fun override(subsystemOverriding: Boolean) = apply { overriding = subsystemOverriding }
    /**
     * Sets whether the task is [muted] and will not report to the Scheduler.
     */
    infix fun mute(schedulerSuppression: Boolean) = apply { muted = schedulerSuppression }

    /**
     * Runs once when the task is initialised.
     */
    infix fun init(onInitialise: Runnable) = apply { init = { onInitialise.run() } }
    /**
     * Runs periodically while the task is active.
     */
    infix fun loop(periodic: Runnable) = apply { loop = { periodic.run() } }
    /**
     * Returning true will end the task.
     */
    infix fun until(isFinished: BooleanSupplier) = apply { until = { isFinished.asBoolean } }
    /**
     * Runs once when the task is finished.
     */
    infix fun finish(onFinish: Runnable) = apply { finish = { onFinish.run() } }
    /**
     * Runs when the task is interrupted (finished but not by [until]).
     */
    infix fun interrupt(onInterrupt: Runnable) = apply { interrupt = { onInterrupt.run() } }
    /**
     * Runs when the task is reset to its initial state.
     */
    infix fun reset(onReset: Runnable) = apply { reset = { onReset.run() } }

    /**
     * Builds the [Task] instance with the specified parameters.
     */
    fun build(): Task = object : Task() {
        init {
            withName(name)
            withTimeout(timeout)
            if (subsystem != null)
                onSubsystem(subsystem!!, overriding)
            if (muted)
                muteReports()
        }
        override fun init() {
            init.invoke()
        }
        override fun periodic() {
            loop.invoke()
        }
        override fun isTaskFinished(): Boolean {
            return until.invoke()
        }
        override fun onFinish() {
            finish.invoke()
        }
        override fun onInterrupt() {
            interrupt.invoke()
        }
        override fun onReset() {
            reset.invoke()
        }
    }
}