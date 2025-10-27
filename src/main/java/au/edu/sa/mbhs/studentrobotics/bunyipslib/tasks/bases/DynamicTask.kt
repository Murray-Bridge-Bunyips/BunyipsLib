package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases

import dev.frozenmilk.util.cell.LateInitCell
import java.util.function.BiFunction
import java.util.function.BooleanSupplier
import java.util.function.Consumer
import java.util.function.Predicate

/**
 * Dynamic builder pattern implementation for [Task] instances.
 * By default, constructing a new [DynamicTask] will create a no-oping indefinite task.
 *
 * You can convert an existing task to a [DynamicTask] using the `.mutate()` method on Task instances.
 *
 * @author Lucas Bubner, 2024
 * @since 6.1.0
 */
class DynamicTask() : Task() {
    private var init = {}
    private var loop = {}
    private var until = { false }
    private var finish = {}
    private var interrupt = {}
    private var reset = {}

    /**
     * A shared [LateInitCell] that can be used for storing state across task execution phases for the [DynamicTask].
     *
     * This cell is not typed and is late-initialised, so type cast and access this cell carefully.
     */
    @JvmField
    val sharedRef = LateInitCell<Any>()

    init {
        named("Task (dyn.)")
    }

    /**
     * Runs once when the task is initialised.
     */
    infix fun init(onInitialise: Consumer<DynamicTask>) = apply { init = { onInitialise.accept(this) } }

    /**
     * Runs periodically while the task is active.
     */
    infix fun periodic(periodic: Consumer<DynamicTask>) = apply { loop = { periodic.accept(this) } }

    /**
     * Returning true will end the task.
     */
    infix fun isFinished(isTaskFinished: java.util.function.Function<DynamicTask, Boolean>) =
        apply { until = { isTaskFinished.apply(this) } }

    /**
     * Runs once when the task is finished.
     */
    infix fun onFinish(onFinish: Consumer<DynamicTask>) = apply { finish = { onFinish.accept(this) } }

    /**
     * Runs when the task is interrupted (finished but not by [isFinished]).
     */
    infix fun onInterrupt(onInterrupt: Consumer<DynamicTask>) = apply { interrupt = { onInterrupt.accept(this) } }

    /**
     * Runs when the task is reset to its initial state.
     */
    infix fun onReset(onReset: Consumer<DynamicTask>) = apply { reset = { onReset.accept(this) } }

    /**
     * Adds additional [init] to run after the current [init] code.
     */
    infix fun addInit(onInitialise: Consumer<DynamicTask>) =
        apply { val f = init; init = { f.invoke(); onInitialise.accept(this) } }

    /**
     * Adds additional [periodic] code to run after the current [periodic] code.
     */
    infix fun addPeriodic(periodic: Consumer<DynamicTask>) =
        apply { val f = loop; loop = { f.invoke(); periodic.accept(this) } }

    /**
     * Adds additional [isFinished] code to evaluate alongside the current [isFinished] condition.
     *
     * This function takes in a boolean being the current [isFinished] evaluation, and returns the new [isFinished] evaluation.
     */
    infix fun addIsFinished(isTaskFinished: BiFunction<DynamicTask, Boolean, Boolean>) =
        apply { val f = until; until = { isTaskFinished.apply(this, f.invoke()) } }

    /**
     * Adds additional [onFinish] code to run after the current [onFinish] code.
     */
    infix fun addOnFinish(onFinish: Consumer<DynamicTask>) =
        apply { val f = finish; finish = { f.invoke(); onFinish.accept(this) } }

    /**
     * Adds additional [onInterrupt] code to run after the current [onInterrupt] code.
     */
    infix fun addOnInterrupt(onInterrupt: Consumer<DynamicTask>) =
        apply { val f = interrupt; interrupt = { f.invoke(); onInterrupt.accept(this) } }

    /**
     * Adds additional [onReset] code to run after the current [onReset] code.
     */
    infix fun addOnReset(onReset: Consumer<DynamicTask>) =
        apply { val f = reset; reset = { f.invoke(); onReset.accept(this) } }

    // Task-injection-less overloads
    // We need to include extra dummy parameters so the Kotlin compiler can pick the correct
    // overload and always expose parameters. We choose to use java.util.function interfaces as we are a Java-first library.

    /**
     * Runs once when the task is initialised.
     */
    @JvmOverloads
    fun init(onInitialise: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        init { _ -> onInitialise.run() }

    /**
     * Runs periodically while the task is active.
     */
    @JvmOverloads
    fun periodic(periodic: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        periodic { _ -> periodic.run() }

    /**
     * Returning true will end the task.
     */
    @JvmOverloads
    fun isFinished(isTaskFinished: BooleanSupplier, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        isFinished { _ -> isTaskFinished.asBoolean }

    /**
     * Runs once when the task is finished.
     */
    @JvmOverloads
    fun onFinish(onFinish: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        onFinish { _ -> onFinish.run() }

    /**
     * Runs when the task is interrupted (finished but not by [isFinished]).
     */
    @JvmOverloads
    fun onInterrupt(onInterrupt: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        onInterrupt { _ -> onInterrupt.run() }

    /**
     * Runs when the task is reset to its initial state.
     */
    @JvmOverloads
    fun onReset(onReset: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        onReset { _ -> onReset.run() }

    /**
     * Adds additional [init] to run after the current [init] code.
     */
    @JvmOverloads
    fun addInit(onInitialise: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        addInit { _ -> onInitialise.run() }

    /**
     * Adds additional [periodic] code to run after the current [periodic] code.
     */
    @JvmOverloads
    fun addPeriodic(periodic: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        addPeriodic { _ -> periodic.run() }

    /**
     * Adds additional [isFinished] code to evaluate alongside the current [isFinished] condition.
     *
     * This function takes in a boolean being the current [isFinished] evaluation, and returns the new [isFinished] evaluation.
     */
    @JvmOverloads
    fun addIsFinished(
        isTaskFinished: Predicate<Boolean>,
        @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null
    ) =
        addIsFinished { _, b -> isTaskFinished.test(b) }

    /**
     * Adds additional [onFinish] code to run after the current [onFinish] code.
     */
    @JvmOverloads
    fun addOnFinish(onFinish: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        addOnFinish { _ -> onFinish.run() }

    /**
     * Adds additional [onInterrupt] code to run after the current [onInterrupt] code.
     */
    @JvmOverloads
    fun addOnInterrupt(onInterrupt: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        addOnInterrupt { _ -> onInterrupt.run() }

    /**
     * Adds additional [onReset] code to run after the current [onReset] code.
     */
    @JvmOverloads
    fun addOnReset(onReset: Runnable, @Suppress("UNUSED_PARAMETER") ktCompatUnusedIgnore: Any? = null) =
        addOnReset { _ -> onReset.run() }

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
        sharedRef.invalidate()
    }
}