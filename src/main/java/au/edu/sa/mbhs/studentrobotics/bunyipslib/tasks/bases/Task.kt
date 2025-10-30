package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Not
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.IncrementingTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.ParallelTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.RaceTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.SequentialTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions.getCallingUserCodeFunction
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import java.util.Optional
import java.util.function.BooleanSupplier

/**
 * A task, or command is an action that can be performed by a robot. This has been designed
 * to reflect closely the command-based programming style used in FRC, while still being
 * reflective of the past nature of how the Task system was implemented in BunyipsLib.
 *
 * The task system in BunyipsLib has been constructed from the ground-up with a more self-sufficient
 * where Tasks are at their core stripped into running some code for some time, somewhere, where all conditions for
 * state are handled internally and not by an external scheduler. The original behaviour of tasks running
 * outright used to be the legacy roots of how Tasks worked, and since has adopted some command-based structures that work
 * by running them in the contexts of other subsystems.
 *
 * As of 6.0.0, the Task system now implements the [Action] interface for seamless compatibility with RoadRunner v1.0.0.
 * The [ActionTask] may be used to convert a pure [Action] into a task, however, writing a task directly using a
 * Task is encouraged for more control and flexibility, due to there being no downsides to doing so and the exposure
 * of the same accessors. Additional changes have also been made to allow for simpler construction of tasks, and
 * the introduction of a new [DynamicTask] class to allow for more flexible task construction.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
abstract class Task : Runnable, Action {
    // By default, task names will be spliced with spaces and Task removed, such that a task with the name MoveToPosTask
    // will have the name "Move To Pos"
    private var name = javaClass.simpleName
        .replace(Regex("([a-z])([A-Z])"), "$1 $2")
        .replace(Regex("Task$"), "")
        .trim()
    private var _dependency: BunyipsSubsystem? = null
    private var attached = false
    private var setConditionFinished = false
    private var startTime = 0L

    /**
     * Enabling will reject all future [on] calls.
     * Useful for composing tasks that will internally re-schedule a wrapped Task.
     *
     * @since 7.0.0
     */
    @JvmField
    protected var disableSubsystemAttachment = false

    /**
     * Maximum timeout of the task. If set to 0 magnitude (or a timeout-less constructor) this will serve as an indefinite task, and
     * will only finish when [isTaskFinished] returns true, or this task is manually interrupted via [finish]/[finish].
     *
     * By default, tasks have an infinite timeout, and will only finish on user conditions.
     *
     * It is encouraged unless it is for systems that require infinite tasks, that you add timeouts to ensure your tasks
     * don't get stuck, as sometimes tolerances can be exceeded and tasks can stall indefinitely.
     */
    @JvmField
    var timeout: Measure<Time> = INFINITE_TIMEOUT

    /**
     * Whether this task should override other tasks in the queue if they conflict with this task. Will only
     * apply if this task has a dependency to run on (see [dependency]).
     */
    @JvmField
    var isPriority: Boolean = false

    /**
     * Get the subsystem reference that this task has elected a dependency on.
     * Will return an Optional where if it is not present, this task is not dependent on any subsystem.
     *
     * See [on].
     */
    val dependency: Optional<BunyipsSubsystem>
        get() = Optional.ofNullable(_dependency)

    /**
     * Whether the task is currently running/active (i.e. has been started ([init] called) and not finished).
     */
    val isActive: Boolean
        get() = startTime != 0L && !isFinished

    /**
     * Time since the task was started.
     */
    val elapsedTime: Measure<Time>
        get() {
            if (startTime == 0L)
                return Nanoseconds.of(0.0)
            return Nanoseconds.of(System.nanoTime().toDouble() - startTime)
        }

    /**
     * Whether the task has been finished and the finisher has fired since the last call to [run]/[execute].
     *
     * As of BunyipsLib 8.0.0, the stateful update of the finished state is now handled fully by the [run] cycle.
     */
    var isFinished = false
        private set

    /**
     * Convenience field to get a reference to a [TelemetryPacket] for sending telemetry to the dashboard.
     * Available as soon as [init] has been called for this task.
     *
     * This field is used for seamless transition between [Action] and [Task] implementations.
     * Use `dashboard.fieldOverlay()` for accessing a field overlay.
     *
     * @since 7.0.0
     */
    protected open lateinit var dashboard: TelemetryPacket

    /**
     * Set the subsystem you want to elect this task to run on, notifying the runner that this task should run there.
     *
     * @param subsystem The subsystem to elect as the runner of this task stored in [dependency]
     * @param override  Whether this task should override conflicting tasks on this subsystem (not incl. default tasks), stored in [isPriority]
     * @return this task
     */
    fun on(subsystem: BunyipsSubsystem, override: Boolean) = apply {
        if (disableSubsystemAttachment) {
            val function = getCallingUserCodeFunction()
            Dbg.error(
                function,
                "[%] This task are not designed to be attached to a subsystem, as it has declared it will be handling subsystem scheduling internally. The on() call should be removed for this wrapped task.",
                javaClass.simpleName
            )
            DualTelemetry.smartLog(
                function, Text.html().color("red", "error: ")
                    .text("${javaClass.simpleName} instance should not be attached to subsystems!")
            )
            return@apply
        }
        _dependency = subsystem
        isPriority = override
    }

    /**
     * Set the subsystem you want to elect this task to run on, notifying the runner that this task should run there.
     * This task is scheduled with default override behaviour (where this task is not priority).
     *
     * @param subsystem The subsystem to elect as the runner of this task
     * @return this task
     */
    infix fun on(subsystem: BunyipsSubsystem) = on(subsystem, false)

    /**
     * Declares that this task should override conflicting tasks on the current [dependency] (not incl. default tasks).
     *
     * May need to be called if your subsystem is ignoring changing to this task as another task is running, but you wish to override it.
     *
     * @since 7.3.0
     */
    fun asPriority() = apply { isPriority = true }

    /**
     * Set the name of this task to be displayed in the OpMode.
     * You may override this method if required to enforce a naming convention/prefix.
     * Null values are ignored.
     */
    open infix fun named(name: String?) = apply { name?.let { this.name = it } }

    /**
     * Set the timeout of this task dynamically and return the task.
     * Null values are ignored.
     */
    infix fun timeout(timeout: Measure<Time>?) = apply { timeout?.let { this.timeout = timeout } }

    /**
     * Get the name of this task. By default, it will be the class simple name, but you can call [named] to set a
     * custom name.
     *
     * @return String representing the name of this task.
     */
    final override fun toString(): String = name

    /**
     * Get a verbose string representation of this task, including all of its properties.
     */
    fun toVerboseString(): String {
        val priority = if (isPriority) "priority, " else ""
        val state = when {
            isFinished -> "finished"
            isActive -> elapsedTime to Seconds round 2
            else -> "ready"
        }
        val time = if (timeout.magnitude() <= 0.0) "∞" else timeout to Seconds round 2
        val dep = if (_dependency != null) ", dependent on ${_dependency.toString()}" else ""

        return Text.format("(%, %t=%/%s%)", name, priority, state, time, dep)
    }

    /**
     * Define code to run once, when the task is started.
     * Override to implement.
     */
    protected open fun init() {
        // no-op
    }

    /**
     * Ensures, and if required, initialises this task calling [init].
     *
     * Also attempts to attach this task to the [dependency] unless [direct] is true (default false).
     */
    @JvmOverloads
    fun ensureInit(direct: Boolean = false) {
        if (startTime != 0L) return
        if (!direct && dependency.isPresent) {
            // Will be recalled immediately with direct=true by setCurrentTask
            attached = dependency.get().setCurrentTask(this)
            return
        }
        Dashboard.usePacket {
            dashboard = it
            Exceptions.runUserMethod(::init)
            startTime = System.nanoTime()
        }
    }

    /**
     * To run as an active loop during this task's duration.
     * Override to implement.
     */
    protected open fun periodic() {
        // no-op
    }

    /**
     * Synchronises this task with the properties of the [sync] task.
     *
     * This reflects the name, timeout, priority, and subsystem (if [disableSubsystemAttachment] is false) of [sync]
     * into this instance.
     */
    protected fun sync(sync: Task) {
        // Two-way sync
        if (sync.isPriority) isPriority = true
        else if (isPriority) sync.isPriority = true
        // Reflect [sync] onto this
        named(sync.toString())
        timeout = sync.timeout
        if (sync.dependency.isPresent && !disableSubsystemAttachment)
            on(sync.dependency.get())
    }

    /**
     * Scheduling method that will either attach this task to a subsystem or run it as part of the standard
     * [run] method. This is the recommended way to run a task as it will attempt to run on the context as provided
     * by the [dependency].
     *
     * A call to [execute] will attempt to schedule the task on every iteration of the [execute] call. As such, if it is rejected,
     * re-scheduling will be attempted as the state of the subsystem may change and allow this task to become scheduled.
     * An already scheduled task on a subsystem will no-op this call. A task with no dependency makes this method the equivalent of [run].
     *
     * @since 7.0.0
     */
    fun execute() {
        if (attached) return
        dependency.ifPresentOrElse({
            if (it.isDisabled) {
                finish()
                return@ifPresentOrElse
            }
            attached = it.setCurrentTask(this)
        }, this::run)
    }

    /**
     * Execute one cycle of this task. As of BunyipsLib 8.0.0, this method will handle all method callback invocation,
     * apart from [finish] which is internally called.
     *
     * Note that this method will perform one cycle of work for this task in the context it is currently,
     * if you wish to respect the [dependency], use the [execute] method.
     */
    final override fun run() {
        dependency.ifPresentOrElse(
            { attached = it.currentTask == this },
            { attached = false })
        if (isFinished) {
            // No work to do
            return
        }
        Dashboard.usePacket {
            dashboard = it
            if (startTime == 0L) {
                // First call, initialise everything
                Exceptions.runUserMethod(::init)
                startTime = System.nanoTime()
            }
            // Do some work
            Exceptions.runUserMethod(::periodic)
            // Update isFinished state
            val timeoutFinished = timeout.magnitude() != 0.0 && System.nanoTime() > startTime + (timeout to Nanoseconds)
            Exceptions.runUserMethod { setConditionFinished = isTaskFinished() }
            if (setConditionFinished || timeoutFinished)
                finish()
        }
    }

    /**
     * RoadRunner [Action] implementation to run this Task.
     *
     * Directly [run]s this task, to avoid recursive cycles.
     */
    final override fun run(p: TelemetryPacket): Boolean {
        // FtcDashboard parameters are handled by the periodic method of this task, so we can ignore the packet
        // and fieldOverlay here as we will handle them ourselves through the `dashboard` field
        run()
        return !isFinished
    }

    /**
     * RoadRunner [Action] implementation to preview the action on the field overlay.
     * This method no-ops as previews are done via DualTelemetry and the Drawing util.
     */
    final override fun preview(fieldOverlay: Canvas) {
        // no-op
    }

    /**
     * Finalising function to run once the task is finished. This will always run regardless of whether
     * the task was ended because of an interrupt or the task naturally finishing.
     * Override to add your own callback.
     */
    protected open fun onFinish() {
        // no-op
    }

    /**
     * Finalising function that will be called after [onFinish] in the event this task is finished via
     * a call to [finish], or if the [timeout] exceeded.
     * Override to add your own callback.
     */
    protected open fun onInterrupt() {
        // no-op
    }

    /**
     * Return a boolean to this method to add custom criteria if a task should be considered finished.
     * @return bool expression indicating whether the task is finished or not, timeout and OpMode state are handled automatically.
     */
    protected open fun isTaskFinished(): Boolean {
        // By default, tasks finish only on timeout
        return false
    }

    /**
     * Called when the task is resetting now. Override this method to add custom reset behaviour, such as resetting any
     * internal state variables such as iterators or lists.
     */
    protected open fun onReset() {
        // no-op
    }

    /**
     * Reset a task to an uninitialised and unfinished state.
     * Will no-op if the task is already fully reset.
     */
    fun reset() {
        if (startTime == 0L && !isFinished)
            return
        Exceptions.runUserMethod(::onReset)
        startTime = 0L
        setConditionFinished = false
        isFinished = false
        attached = false
    }

    /**
     * Causes a task to end and fire finishing methods.
     */
    fun finish() {
        if (isFinished) return
        isFinished = true
        Exceptions.runUserMethod(::onFinish)
        if (!setConditionFinished)
            Exceptions.runUserMethod(::onInterrupt)
        dependency.ifPresent {
            if (attached && it.currentTask == this) {
                attached = false
                it.cancelCurrentTask()
            }
        }
    }

    /**
     * Attempts to assign this task as the default task of the current [dependency].
     *
     * If a [dependency] is not attached to this task, meaning that this task has not elected a subsystem to run this task on
     * via [on], this method will no-op and throw a warning in Logcat. Most BunyipsLib-integrated tasks, including ones
     * found in the `tasks` field of subsystems is elected to run on the correct [dependency], so this method can be used.
     *
     * Note that an assigned default task must not have a finish condition, otherwise an emergency stop will be issued.
     *
     * @since 7.0.1
     */
    fun setAsDefaultTask() {
        val dep = _dependency
        if (dep == null) {
            Dbg.warn(javaClass, "tried assigning this task (%) to a dependency when none was present!", this)
            return
        }
        dep default this
    }

    /**
     * Compose this task into a [RaceTaskGroup] with a wait condition based on this condition.
     */
    infix fun until(condition: BooleanSupplier): RaceTaskGroup {
        val task = task { isFinished { condition.asBoolean } }
        task.named("until $condition")
        return RaceTaskGroup(this, task)
    }

    /**
     * Compose this task into a [RaceTaskGroup] with a wait condition based on this negation of this condition.
     */
    infix fun onlyWhile(condition: BooleanSupplier) = until(Not(condition))

    /**
     * Compose this task into a [DeadlineTaskGroup] to run alongside until
     * the supplied task is done.
     */
    infix fun until(deadline: Task) = DeadlineTaskGroup(deadline, this)

    /**
     * Composes a [ParallelTaskGroup] with a [WaitTask] to run before this task.
     * This will ensure the task runs for at least the specified time, and no-ops until the duration if it finishes early.
     */
    infix fun forAtLeast(waitTime: Measure<Time>): ParallelTaskGroup {
        val task = WaitTask(waitTime)
        task.named("for at least $task")
        return ParallelTaskGroup(this, task)
    }

    /**
     * Composes a [ParallelTaskGroup] with a [WaitTask] to run before this task.
     * This will ensure the task runs for at least the specified time, and no-ops until the duration if it finishes early.
     */
    fun forAtLeast(waitDuration: Double, unit: Time) = forAtLeast(unit.of(waitDuration))

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied
     * tasks to run before this one.
     */
    fun after(vararg otherTasks: Task) = SequentialTaskGroup(*otherTasks, this)

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied task
     * to run before this one.
     */
    infix fun after(otherTask: Task) = SequentialTaskGroup(otherTask, this)

    /**
     * Composes a [WaitTask] to run before this task.
     */
    infix fun after(waitTime: Measure<Time>): SequentialTaskGroup {
        val task = WaitTask(waitTime)
        task.named("after $task")
        return SequentialTaskGroup(task, this)
    }

    /**
     * Composes a [WaitTask] to run before this task.
     */
    fun after(waitDuration: Double, unit: Time) = after(unit.of(waitDuration))

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable] named [name],
     * queued to run before this task starts.
     */
    fun after(runnable: Runnable, name: String = "Callback"): SequentialTaskGroup {
        val task = Lambda(runnable).named(name)
        task.named("after $task")
        return SequentialTaskGroup(task, this)
    }

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable],
     * queued to run before this task starts.
     */
    infix fun after(runnable: Runnable) = after(runnable, "Callback")

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied tasks
     * to follow after this one.
     */
    fun then(vararg otherTasks: Task) = SequentialTaskGroup(this, *otherTasks)

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied task
     * to follow after this one.
     */
    infix fun then(otherTask: Task) = SequentialTaskGroup(this, otherTask)

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable] named [name],
     * queued to run when this task finishes.
     */
    fun then(runnable: Runnable, name: String = "Callback"): SequentialTaskGroup {
        val task = Lambda(runnable).named(name)
        task.named("then $task")
        return SequentialTaskGroup(this, task)
    }

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable],
     * queued to run when this task finishes.
     */
    infix fun then(runnable: Runnable) = then(runnable, "Callback")

    /**
     * Compose this task into a [ParallelTaskGroup] with the supplied tasks
     * to run all of these tasks at once.
     */
    fun with(vararg otherTasks: Task) = ParallelTaskGroup(this, *otherTasks)

    /**
     * Compose this task into a [ParallelTaskGroup] with the supplied task
     * to run alongside this one.
     */
    infix fun with(otherTask: Task) = ParallelTaskGroup(this, otherTask)

    /**
     * Compose this task into a [RaceTaskGroup] with the supplied tasks
     * to run all of these tasks until one finishes.
     */
    fun race(vararg otherTasks: Task) = RaceTaskGroup(this, *otherTasks)

    /**
     * Compose this task into a [RaceTaskGroup] with the supplied task
     * to run alongside this one until one finishes.
     */
    infix fun race(otherTask: Task) = RaceTaskGroup(this, otherTask)

    /**
     * Compose this task into a [DeadlineTaskGroup] with the supplied tasks
     * to run these extra tasks until this task is done.
     */
    fun during(vararg otherTasks: Task) = DeadlineTaskGroup(this, *otherTasks)

    /**
     * Compose this task into a [DeadlineTaskGroup] with the supplied task
     * to run alongside this one until this task is done.
     */
    infix fun during(otherTask: Task) = DeadlineTaskGroup(this, otherTask)

    /**
     * Compose this task into a [IncrementingTaskGroup] with the supplied tasks
     * to run the next task in sequence after the previous one finishes, while looping back to the first task.
     */
    fun next(vararg otherTasks: Task) = IncrementingTaskGroup(this, *otherTasks)

    /**
     * Compose this task into a [IncrementingTaskGroup] with the supplied task
     * to run alongside this one until one finishes.
     */
    infix fun next(otherTask: Task) = IncrementingTaskGroup(this, otherTask)

    /**
     * Wrap this task in a [RepeatTask] where finish conditions are reset immediately.
     */
    fun repeatedly(): Task = RepeatTask(this)

    /**
     * Wrap this task in a [ConditionalTask] that will only schedule if [condition] is false.
     */
    fun unless(condition: BooleanSupplier) = ConditionalTask(Lambda(), this, condition)

    /**
     * Wrap this task in a [ConditionalTask] that will only schedule if [condition] is true.
     */
    fun onlyIf(condition: BooleanSupplier) = ConditionalTask(this, Lambda(), condition)

    /**
     * Creates a new [DynamicTask] instance by wrapping this existing [Task] instance, allowing
     * you to add new functionality to a task without modifying the original task.
     *
     * This method will return the current task if it is already a [DynamicTask] instance.
     *
     * @since 7.0.0
     */
    fun mutate(): DynamicTask {
        return this as? DynamicTask ?: task {
            named(this@Task.name)
            timeout(this@Task.timeout)
            if (this@Task.dependency.isPresent)
                on(this@Task.dependency.get(), this@Task.isPriority)
            isPriority = this@Task.isPriority
            // We shim away the use of run() and execute(), since the DynamicTask will be the new task that is
            // scheduled onto subsystems. All logic relating to timeouts is handled by the parent wrapping DynamicTask.
            // The only things we need to pass forward are things the user has adjusted, which include the function
            // definitions for init, periodic, interrupt, reset, finish, and the finish condition.
            init {
                this@Task.dashboard = dashboard // Need to assign for the inner instance which may reference `dashboard`
                this@Task.init()
            }
            periodic {
                this@Task.dashboard = dashboard
                this@Task.periodic()
            }
            isFinished { this@Task.isTaskFinished() }
            onInterrupt { this@Task.onInterrupt() }
            onReset { this@Task.onReset() }
            onFinish { this@Task.onFinish() }
        }
    }

    companion object {
        /**
         * Timeout value for an infinite task that will run forever.
         */
        @JvmField
        val INFINITE_TIMEOUT: Measure<Time> = Seconds.zero()

        /**
         * Constructor utility to create a new [DeferredTask] based on the supplied task builder.
         *
         * Useful for constructing tasks that use data that is not available at the build time of the wrapped task.
         */
        @JvmStatic
        fun defer(taskBuilder: () -> Task) = DeferredTask(taskBuilder)

        /**
         * Shorthand group utility for creating a [SequentialTaskGroup].
         */
        @JvmStatic
        fun seq(vararg tasks: Task) = SequentialTaskGroup(*tasks)

        /**
         * Shorthand group utility for creating a [ParallelTaskGroup].
         */
        @JvmStatic
        fun par(vararg tasks: Task) = ParallelTaskGroup(*tasks)

        /**
         * Shorthand group utility for creating a [RaceTaskGroup].
         */
        @JvmStatic
        fun rce(vararg tasks: Task) = RaceTaskGroup(*tasks)

        /**
         * Shorthand group utility for creating a [DeadlineTaskGroup].
         */
        @JvmStatic
        fun ddl(vararg tasks: Task) = DeadlineTaskGroup(*tasks)

        /**
         * Shorthand group utility for creating a [IncrementingTaskGroup].
         */
        @JvmStatic
        fun inc(vararg tasks: Task) = IncrementingTaskGroup(*tasks)

        /**
         * Utility for creating a [DynamicTask] that will loop some [function] for some [time].
         */
        @JvmStatic
        fun runFor(time: Measure<Time>, function: Runnable) =
            task { named("$time loop (dyn.)"); timeout(time); periodic(function) }

        /**
         * Utility for creating a [DynamicTask] that will loop some [function] until the task is manually finished.
         */
        @JvmStatic
        fun loop(function: Runnable) = task { named("Loop (dyn.)"); periodic(function) }

        /**
         * Utility for creating a [DynamicTask] that will wait for some [condition] and finish when the condition is true.
         */
        @JvmStatic
        fun waitFor(condition: BooleanSupplier) = task { named("Busy wait (dyn.)"); isFinished(condition) }

        /**
         * Utility to create a new [DynamicTask] instance for building a new task.
         */
        @JvmStatic
        fun task() = DynamicTask()

        /**
         * Utility to cast a task into the desired task class.
         * Useful for ignoring task nullability to cast into a new type - ensure your task exists before calling.
         */
        @JvmStatic
        fun <T : Task> cast(task: Task?, cast: Class<T>) = cast.cast(task) as T

        /**
         * DSL function to create a new [DynamicTask] instance for building a new task.
         */
        fun task(block: DynamicTask.() -> Unit) = DynamicTask().apply(block)

        /**
         * Default task setter extension for [BunyipsSubsystem] to set the default task of a subsystem.
         */
        infix fun BunyipsSubsystem.default(defaultTask: Task) = setDefaultTask(defaultTask)
    }
}