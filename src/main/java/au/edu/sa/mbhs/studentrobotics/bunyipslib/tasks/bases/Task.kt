package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsComponent
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ActionTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.DeferredTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.Lambda
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.RepeatTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitUntilTask
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.ParallelTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.RaceTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.SequentialTaskGroup
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard
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
 * The task system in BunyipsLib has been constructed from the ground-up with a more lightweight ecosystem
 * where Tasks are at their core stripped into running some code for some time, somewhere. The original behaviour of tasks running
 * outright used to be the legacy roots of how Tasks worked, and since has adopted some command-based structures that work
 * alongside the lightweight premise of a Task being "a Runnable with a timeout" with a run implementation left to the user.
 *
 * Some different implementations on how Tasks are interpreted are demonstrated in these classes:
 * - Scheduler
 * - BunyipsOpMode
 * - AutonomousBunyipsOpMode
 * - BunyipsSubsystem
 * - Tasks
 *
 * Task extends [BunyipsComponent] to allow for simpler integration with accessing the OpMode, and was a legacy
 * feature that was kept for the sake of simplicity, more pedantic exception handling, and ease of use.
 *
 * As of 6.0.0, the Task system now implements the [Action] interface for seamless compatibility with RoadRunner v1.0.0.
 * The [ActionTask] may be used to convert a pure [Action] into a task, however, writing a task directly using a
 * Task is encouraged for more control and flexibility, due to there being no downsides to doing so and the exposure
 * of the same accessors.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
abstract class Task(
    /**
     * Maximum timeout of the task. If set to 0 magnitude (or a timeout-less constructor) this will serve as an indefinite task, and
     * will only finish when [isTaskFinished] returns true, or this task is manually interrupted via [finish].
     *
     * It is encouraged unless it is for systems that require infinite tasks, that you add timeouts to ensure your tasks
     * don't get stuck, making tasks have a focus on timeout conditions.
     */
    var timeout: Measure<Time>
) : BunyipsComponent(), Runnable, Action {
    private var overrideDependency: Boolean = false
    private var dependency: BunyipsSubsystem? = null
    private var mutedReport = false
    private var name = this.javaClass.simpleName

    // RoadRunner Actions compatibility, exposes the same field names as they are presented in the Action interface
    /**
     * Convenience field to get a reference to FtcDashboard's field overlay for drawing on the field.
     * Available as soon as init() has been called for this task.
     *
     * This field is used for porting between [Action] and [Task] implementations.
     */
    protected lateinit var fieldOverlay: Canvas

    /**
     * Convenience field to get a reference to a [TelemetryPacket] for sending telemetry to the dashboard.
     * Available as soon as init() has been called for this task.
     *
     * This field is used for porting between [Action] and [Task] implementations.
     */
    protected lateinit var p: TelemetryPacket

    /**
     * Set the subsystem you want to elect this task to run on, notifying the runner that this task should run there.
     *
     * @param subsystem The subsystem to elect as the runner of this task
     * @param override  Whether this task should override conflicting tasks on this subsystem (not incl. default tasks)
     * @return this task
     */
    @JvmOverloads
    open fun onSubsystem(subsystem: BunyipsSubsystem, override: Boolean = false): Task {
        dependency = subsystem
        overrideDependency = override
        return this
    }

    /**
     * Set the subsystem you want to elect this task to run on, notifying the runner that this task should run there.
     * This task is scheduled with default override behaviour.
     *
     * @param subsystem The subsystem to elect as the runner of this task
     * @return this task
     */
    infix fun on(subsystem: BunyipsSubsystem): Task {
        return onSubsystem(subsystem)
    }

    /**
     * Return whether this task has elected a dependency on a subsystem or not.
     */
    fun hasDependency(): Boolean {
        return dependency != null
    }

    /**
     * Get the subsystem reference that this task has elected a dependency on.
     * Will return an Optional where if it is not present, this task is not dependent on any subsystem.
     */
    fun getDependency(): Optional<BunyipsSubsystem> {
        return Optional.ofNullable(dependency)
    }

    /**
     * Mute task reports from the Scheduler.
     */
    fun muteReports(): Task {
        mutedReport = true
        return this
    }

    /**
     * @return Whether this task is muted from subsystem reports or not.
     */
    fun isMuted(): Boolean {
        return mutedReport
    }

    /**
     * @return Whether this task should override other tasks in the queue if they conflict with this task. Will only
     *         apply if this task has a dependency (see [hasDependency], [getDependency]).
     */
    fun isOverriding(): Boolean {
        return overrideDependency
    }

    /**
     * A task that does not have an integrated timeout, and will rely on manual intervention and [isTaskFinished].
     */
    constructor() : this(INFINITE_TIMEOUT)

    /**
     * Set the name of this task to be displayed in the OpMode.
     * You may override this method if required to enforce a naming convention/prefix.
     */
    open fun withName(name: String?): Task {
        if (name == null) {
            return this
        }
        this.name = name
        return this
    }

    /**
     * Set the name of this task to be displayed in the OpMode.
     */
    infix fun named(name: String): Task {
        return withName(name)
    }

    /**
     * Get the name of this task. By default, it will be the class simple name, but you can call [withName] to set a
     * custom name.
     *
     * @return String representing the name of this task.
     */
    final override fun toString(): String {
        return name
    }

    /**
     * Get a verbose string representation of this task, including all of its properties.
     */
    fun toVerboseString(): String {
        return name + "[${if (taskFinished) "FINISHED" else "READY"}, ${if (isRunning) deltaTime else "NOT RUNNING"}/${if (timeout.magnitude() == 0.0) "INDEFINITE" else "$timeout"}, ${if (dependency != null) "DEPENDENT ON <${dependency?.toString()}>" else "INDEPENDENT"}, ${if (overrideDependency) "OVERRIDING" else "NON-OVERRIDING"}, ${if (mutedReport) "MUTED" else "REPORTING"}]"
    }

    /**
     * Set the timeout of this task dynamically and return the task.
     */
    fun withTimeout(timeout: Measure<Time>): Task {
        this.timeout = timeout
        return this
    }

    /**
     * Set the timeout of this task dynamically and return the task.
     */
    infix fun timeout(timeout: Measure<Time>): Task {
        return withTimeout(timeout)
    }

    /**
     * Compose this task into a [RaceTaskGroup] with a [WaitUntilTask] based on this condition.
     */
    infix fun until(condition: BooleanSupplier): RaceTaskGroup {
        val task = WaitUntilTask(condition)
        task.withName("$name supervisor")
        return RaceTaskGroup(this, task)
    }

    /**
     * Composes a [ParallelTaskGroup] with a [WaitTask] to run before this task.
     * This will ensure the task runs for at least the specified time, and no-ops until the duration if it finishes early.
     */
    infix fun forAtLeast(waitTime: Measure<Time>): ParallelTaskGroup {
        val task = WaitTask(waitTime)
        task.withName("$name wait")
        return ParallelTaskGroup(this, task)
    }

    /**
     * Composes a [ParallelTaskGroup] with a [WaitTask] to run before this task.
     * This will ensure the task runs for at least the specified time, and no-ops until the duration if it finishes early.
     */
    fun forAtLeast(waitDuration: Double, unit: Time): ParallelTaskGroup {
        return forAtLeast(unit.of(waitDuration))
    }

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied
     * tasks to run before this one.
     */
    fun after(vararg otherTasks: Task): SequentialTaskGroup {
        return SequentialTaskGroup(*otherTasks, this)
    }

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied task
     * to run before this one.
     */
    infix fun after(otherTask: Task): SequentialTaskGroup {
        return SequentialTaskGroup(otherTask, this)
    }

    /**
     * Composes a [WaitTask] to run before this task.
     */
    infix fun after(waitTime: Measure<Time>): SequentialTaskGroup {
        val task = WaitTask(waitTime)
        task.withName("$name wait")
        return SequentialTaskGroup(task, this)
    }

    /**
     * Composes a [WaitTask] to run before this task.
     */
    fun after(waitDuration: Double, unit: Time): SequentialTaskGroup {
        return after(unit.of(waitDuration))
    }

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable],
     * queued to run before this task starts.
     */
    infix fun after(runnable: Runnable): SequentialTaskGroup {
        val task = Lambda(runnable)
        task.withName("$name hook")
        return SequentialTaskGroup(task, this)
    }

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied tasks
     * to follow after this one.
     */
    fun then(vararg otherTasks: Task): SequentialTaskGroup {
        return SequentialTaskGroup(this, *otherTasks)
    }

    /**
     * Compose this task into a [SequentialTaskGroup] with the supplied task
     * to follow after this one.
     */
    infix fun then(otherTask: Task): SequentialTaskGroup {
        return SequentialTaskGroup(this, otherTask)
    }

    /**
     * Implicitly run a [SequentialTaskGroup] with this supplied [Runnable],
     * queued to run when this task finishes.
     */
    infix fun then(runnable: Runnable): SequentialTaskGroup {
        val task = Lambda(runnable)
        task.withName("$name callback")
        return SequentialTaskGroup(this, task)
    }

    /**
     * Compose this task into a [ParallelTaskGroup] with the supplied tasks
     * to run all of these tasks at once.
     */
    fun with(vararg otherTasks: Task): ParallelTaskGroup {
        return ParallelTaskGroup(this, *otherTasks)
    }

    /**
     * Compose this task into a [ParallelTaskGroup] with the supplied task
     * to run alongside this one.
     */
    infix fun with(otherTask: Task): ParallelTaskGroup {
        return ParallelTaskGroup(this, otherTask)
    }

    /**
     * Compose this task into a [RaceTaskGroup] with the supplied tasks
     * to run all of these tasks until one finishes.
     */
    fun race(vararg otherTasks: Task): RaceTaskGroup {
        return RaceTaskGroup(this, *otherTasks)
    }

    /**
     * Compose this task into a [RaceTaskGroup] with the supplied task
     * to run alongside this one until one finishes.
     */
    infix fun race(otherTask: Task): RaceTaskGroup {
        return RaceTaskGroup(this, otherTask)
    }

    /**
     * Compose this task into a [DeadlineTaskGroup] with the supplied tasks
     * to run these extra tasks until this task is done.
     */
    fun during(vararg otherTasks: Task): DeadlineTaskGroup {
        return DeadlineTaskGroup(this, *otherTasks)
    }

    /**
     * Compose this task into a [DeadlineTaskGroup] with the supplied task
     * to run alongside this one until this task is done.
     */
    infix fun during(otherTask: Task): DeadlineTaskGroup {
        return DeadlineTaskGroup(this, otherTask)
    }

    /**
     * Wrap this task in a [RepeatTask] where finish conditions are reset immediately.
     */
    fun repeatedly(): Task {
        return RepeatTask(this)
    }

    /**
     * Whether the task is finished or not via timeout or custom condition. Will be true regardless of the finisher
     * being fired or not, as some tasks will handle this via finishNow().
     */
    @Volatile
    var taskFinished = false
        private set

    private var startTime = 0L
    private var finisherFired = false

    /**
     * Define code to run once, when the task is started.
     * Override to implement.
     */
    protected open fun init() {
        // no-op
    }

    /**
     * To run as an active loop during this task's duration.
     */
    protected abstract fun periodic()

    /**
     * Should be called by your polling loop to run the task and manage all state properly.
     */
    final override fun run() {
        Dashboard.usePacket {
            p = it
            fieldOverlay = it.fieldOverlay()
            if (startTime == 0L) {
                Exceptions.runUserMethod(::init, opMode)
                startTime = System.nanoTime()
                // Must poll finished on the first iteration to ensure that the task does not overrun
                pollFinished()
            }
            // Here we check the taskFinished condition but don't call pollFinished(), to ensure that the task is only
            // updated with latest finish information at the user's discretion (excluding the first-call requirement)
            if (taskFinished && !finisherFired) {
                Exceptions.runUserMethod(::onFinish, opMode)
                if (!isFinished())
                    Exceptions.runUserMethod(::onInterrupt, opMode)
                finisherFired = true
            }
            // Don't run the task if it is finished as a safety guard
            if (isFinished()) return@usePacket
            Exceptions.runUserMethod(::periodic, opMode)
        }
    }

    /**
     * RoadRunner [Action] implementation to run this Task.
     */
    final override fun run(p: TelemetryPacket): Boolean {
        // FtcDashboard parameters are handled by the periodic method of this task, so we can ignore the packet
        // and fieldOverlay here as we will handle them ourselves. This also allows Action to Task porting
        // to use the same field parameters.
        run()
        return !pollFinished()
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
     * a call to [finish] or [finishNow].
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
     * Query (but not update) the finished state of the task. This will return true if the task is finished and the
     * finisher has been fired.
     */
    fun isFinished(): Boolean {
        return taskFinished && finisherFired
    }

    /**
     * Update and query the state of the task if it is finished. This will return true if the task is finished and the
     * finisher has been fired.
     */
    fun pollFinished(): Boolean {
        // Early return
        if (taskFinished) return finisherFired

        val startCalled = startTime != 0L
        val timeoutFinished = timeout.magnitude() != 0.0 && System.nanoTime() > startTime + (timeout to Nanoseconds)
        var userCondition = false
        Exceptions.runUserMethod({ userCondition = isTaskFinished() }, opMode)

        taskFinished = startCalled && (timeoutFinished || userCondition)

        // run() will handle firing the finisher, in which case we can return true and the polling loop can stop
        return taskFinished && finisherFired
    }

    /**
     * Reset a task to an uninitialised and unfinished state.
     * Will no-op if the task is already fully reset.
     */
    fun reset() {
        if (startTime == 0L && !taskFinished && !finisherFired)
            return
        Exceptions.runUserMethod(::onReset, opMode)
        startTime = 0L
        taskFinished = false
        finisherFired = false
    }

    /**
     * Tell a task to finish on the next iteration.
     */
    fun finish() {
        taskFinished = true
    }

    /**
     * Force a task to finish immediately, and fire the onFinish() method without waiting
     * for the next polling loop. This method is useful when your task needs to die and
     * needs to finish up immediately. If your finisher has already been fired, this method
     * will do nothing but ensure that the task is marked as finished.
     */
    fun finishNow() {
        taskFinished = true
        if (!finisherFired) {
            Exceptions.runUserMethod(::onFinish, opMode)
            Exceptions.runUserMethod(::onInterrupt, opMode)
        }
        finisherFired = true
    }

    /**
     * @return Whether the task is currently running (i.e. has been started (`init()` called) and not finished).
     */
    val isRunning: Boolean
        get() = startTime != 0L && !isFinished()

    /**
     * Time in seconds since the task was started.
     */
    val deltaTime: Measure<Time>
        get() {
            if (startTime == 0L)
                return Nanoseconds.of(0.0)
            return Nanoseconds.of((System.nanoTime() - startTime).toDouble())
        }

    companion object {
        /**
         * Timeout value for an infinite task that will run forever.
         */
        @JvmField
        val INFINITE_TIMEOUT: Measure<Time> = Seconds.zero()

        /**
         * Utility to create a new [DeferredTask] based on the supplied task builder.
         * Useful for constructing tasks that use data that is not available at the build time of the wrapped task.
         */
        @JvmStatic
        fun defer(taskBuilder: () -> Task): DeferredTask {
            return DeferredTask(taskBuilder)
        }

        /**
         * Utility to create a new [DynamicTask] instance for building a new task.
         */
        @JvmStatic
        fun task() = DynamicTask()

        /**
         * DSL function to create a new [DynamicTask] instance for building a new task.
         */
        fun task(block: DynamicTask.() -> Unit) = DynamicTask().apply(block)

        /**
         * Default task setter extension for [BunyipsSubsystem] to set the default task of a subsystem.
         */
        infix fun BunyipsSubsystem.default(defaultTask: Task) {
            this.setDefaultTask(defaultTask)
        }
    }
}