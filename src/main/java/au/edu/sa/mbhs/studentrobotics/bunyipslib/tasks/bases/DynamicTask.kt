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
     * Adds additional [init] to run after the current [init] code.
     */
    infix fun addInit(onInitialise: Runnable) =
        apply { val f = init; init = { f.invoke(); onInitialise.run() } }

    /**
     * Adds additional [periodic] code to run after the current [periodic] code.
     */
    infix fun addPeriodic(periodic: Runnable) =
        apply { val f = loop; loop = { f.invoke(); periodic.run() } }

    /**
     * Adds additional [isFinished] code to evaluate alongside the current [isFinished] condition.
     *
     * This function takes in a boolean being the current [isFinished] evaluation, and returns the new [isFinished] evaluation.
     */
    infix fun addIsFinished(isTaskFinished: java.util.function.Function<Boolean, Boolean>) =
        apply { val f = until; until = { isTaskFinished.apply(f.invoke()) } }

    /**
     * Adds additional [onFinish] code to run after the current [onFinish] code.
     */
    infix fun addOnFinish(onFinish: Runnable) =
        apply { val f = finish; finish = { f.invoke(); onFinish.run() } }

    /**
     * Adds additional [onInterrupt] code to run after the current [onInterrupt] code.
     */
    infix fun addOnInterrupt(onInterrupt: Runnable) =
        apply { val f = interrupt; interrupt = { f.invoke(); onInterrupt.run() } }

    /**
     * Adds additional [onReset] code to run after the current [onReset] code.
     */
    infix fun addOnReset(onReset: Runnable) =
        apply { val f = reset; reset = { f.invoke(); onReset.run() } }

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