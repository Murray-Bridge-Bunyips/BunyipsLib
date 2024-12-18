package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases

import java.util.function.BooleanSupplier

/**
 * Dynamic builder pattern implementation for [Task] instances.
 *
 * @author Lucas Bubner, 2024
 * @since 6.1.0
 */
open class DynamicTask() : Task() {
    private var init = {}
    private var loop = {}
    private var until = { false }
    private var finish = {}
    private var interrupt = {}
    private var reset = {}

    /**
     * Wraps a [Task] instance to expose a dynamic builder pattern, adding additional functionality.
     */
    constructor(task: Task) : this() {
        dyn(task).let {
            init = it.init
            loop = it.loop
            until = it.until
            finish = it.finish
            interrupt = it.interrupt
            reset = it.reset
        }
    }

    /**
     * Runs once when the task is initialised.
     */
    infix fun init(onInitialise: Runnable) = apply { init = { onInitialise.run() } }

    /**
     * Runs periodically while the task is active.
     */
    infix fun periodic(periodic: Runnable) = apply { loop = { periodic.run() } }

    /**
     * Returning true will end the task.
     */
    infix fun isFinished(isTaskFinished: BooleanSupplier) = apply { until = { isTaskFinished.asBoolean } }

    /**
     * Runs once when the task is finished.
     */
    infix fun onFinish(onFinish: Runnable) = apply { finish = { onFinish.run() } }

    /**
     * Runs when the task is interrupted (finished but not by [onFinish]).
     */
    infix fun onInterrupt(onInterrupt: Runnable) = apply { interrupt = { onInterrupt.run() } }

    /**
     * Runs when the task is reset to its initial state.
     */
    infix fun onReset(onReset: Runnable) = apply { reset = { onReset.run() } }

    /**
     * Adds additional init to run after the initialisation code.
     */
    infix fun addInit(onInitialise: Runnable) = apply { init = { init.invoke(); onInitialise.run() } }

    /**
     * Adds additional periodic code to run after the periodic code.
     */
    infix fun addPeriodic(periodic: Runnable) = apply { loop = { loop.invoke(); periodic.run() } }

    /**
     * Adds additional isFinished code to run after the isFinished code.
     *
     * This function takes in a boolean being the current isFinished evaluation, and returns the new isFinished evaluation.
     */
    infix fun addIsFinished(isTaskFinished: java.util.function.Function<Boolean, Boolean>) = apply { val u = until; until = { isTaskFinished.apply(u.invoke()) } }

    /**
     * Adds additional onFinish code to run after the onFinish code.
     */
    infix fun addOnFinish(onFinish: Runnable) = apply { finish = { finish.invoke(); onFinish.run() } }

    /**
     * Adds additional onInterrupt code to run after the onInterrupt code.
     */
    infix fun addOnInterrupt(onInterrupt: Runnable) = apply { interrupt = { interrupt.invoke(); onInterrupt.run() } }

    /**
     * Adds additional onReset code to run after the onReset code.
     */
    infix fun addOnReset(onReset: Runnable) = apply { reset = { reset.invoke(); onReset.run() } }

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