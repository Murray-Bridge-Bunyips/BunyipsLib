package au.edu.sa.mbhs.studentrobotics.bunyipslib.util

import android.util.Pair
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads.startLoop
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads.stopAll
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads.task
import com.qualcomm.robotcore.util.ThreadPool
import java.util.concurrent.Callable
import java.util.concurrent.CancellationException
import java.util.concurrent.ExecutionException
import java.util.concurrent.Future
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.max

/**
 * Async utilities for running user code on different threads while having control/logging over them.
 *
 * **WARNING:** Multithreading is a highly advanced topic where issues such as race conditions can arise.
 * You must be fully aware of the consequences of threading and whether it is necessary for your program. For 99% of use
 * cases, threading is not required for normal robot operation. Also note that there exists a global hardware lock
 * for the SDK, so attempting to multi-thread hardware for loop times will drastically hinder robot performance.
 * Threads are best used for blocking operations that do not access hardware reads and writes.
 *
 * Tasks are scheduled through the SDK's default cached [ThreadPool]. Do note that by executing tasks on this pool,
 * exceptions will not be rethrown to the OpMode thread when required (such as when an `EmergencyStop` is requested).
 * Exceptions are still logged to the DS and Logcat in standard `Exceptions` fashion, and `BunyipsOpMode` tries
 * to look for these exceptions manually to terminate the OpMode, but do note the reduced exception safety in a thread.
 *
 * Threads started via this class are automatically shut down at the end of OpModes, via the [stopAll] method
 * and a BunyipsLib [Hook].
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
object Threads {
    private val tasks = HashMap<String, Pair<Int, Result<*>>>()

    private class ResultImpl<T>(base: Future<T>, override val function: Callable<T>) : Future<T> by base, Result<T> {
        override var ignoreStopAll = false
    }

    /**
     * An extension of [Future] which allows additional configuration of the running task.
     *
     * @param T the result type
     * @author Lucas Bubner, 2025
     * @see Future
     */
    interface Result<T> : Future<T> {
        /**
         * @return the function being executed that was originally passed into the task. If this was from a [startLoop],
         *         it will be the function that will be periodically executed, not the wrapped loop, and exception handling
         *         will be stripped. Note that for [startLoop], `Runnable` will be converted to a null-returning `Callable`.
         */
        val function: Callable<T>

        /**
         * Whether this task should ignore the [stopAll] command which is automated to run at the end
         * of every OpMode via a [Hook]. Defaults to false.
         */
        var ignoreStopAll: Boolean

        /**
         * Waits if necessary for at most the given time for the computation
         * to complete, and then retrieves its result, if available.
         *
         * @param timeout the maximum time to wait using WPIUnits
         * @return the computed result
         * @throws CancellationException if the computation was cancelled
         * @throws ExecutionException if the computation threw an exception
         * @throws InterruptedException if the current thread was interrupted while waiting
         * @throws TimeoutException if the wait timed out
         */
        @Throws(
            CancellationException::class,
            ExecutionException::class,
            InterruptedException::class,
            TimeoutException::class
        )
        fun get(timeout: Measure<Time>): T = get((timeout to Milliseconds).toLong(), TimeUnit.MILLISECONDS)
    }

    /**
     * Start a new thread task with the given [Callable] that may return a result.
     *
     * A result does not need to be utilised (as it can be ignored), but one must be supplied via the
     * [Callable] interface. You can choose to return a null value if there is no results you need.
     *
     * For tasks that run repeatedly, consider [startLoop], which takes in a [Runnable].
     *
     * @param T return type for the returned [Result]
     * @param name the unique name of the task to access it later and to log as
     * @param task the runnable task to run on the new thread
     * @return a [Result] of type T that can be used to retrieve the result of this task
     */
    @JvmStatic
    fun <T> start(name: String, task: Callable<T?>): Result<T?> {
        require(tasks.all { it.key != name || it.value.second.isDone }) {
            "name parameter '$name' is already being used as an active Threads task name; it must be unique"
        }
        val t = Callable {
            try {
                return@Callable task.call()
            } catch (e: Exception) {
                Exceptions.handle(e) { DualTelemetry.smartLog(it) }
            } finally {
                Dbg.logd(Threads::class.java, "thread task '%(%)' completed.", name, task.hashCode())
            }
            null
        }
        val scheduled = ResultImpl(ThreadPool.getDefault().submit(t), task)
        tasks[name] = Pair(task.hashCode(), scheduled)
        Dbg.logd(Threads::class.java, "starting new thread task: %(%) ...", name, task.hashCode())
        return scheduled
    }

    /**
     * Start a new thread task with the given infinite loop task. These tasks are not designed to return any results,
     * although a [Result] will be returned to you if required.
     *
     * This will auto-end when the task is interrupted.
     *
     * @param name        the unique name of the task to access it later and to log as
     * @param minLoopTime the minimum duration this loop should run at to save resources, will be slower if your loop takes longer than the minimum
     * @param loopTask    the infinite loop task to run on the new thread, contains no result as it is an infinite loop
     * @return an unbounded [Result] for the task, note this will never naturally get a result
     */
    @JvmStatic
    fun startLoop(name: String, minLoopTime: Measure<Time>, loopTask: Runnable): Result<*> {
        require(tasks.all { it.key != name || it.value.second.isDone }) {
            "name parameter '$name' is already being used as an active Threads task name; it must be unique"
        }
        val magMillis = (minLoopTime to Milliseconds).toLong()
        val t = Callable {
            try {
                while (!Thread.currentThread().isInterrupted) {
                    val start = System.currentTimeMillis()
                    try {
                        loopTask.run()
                    } catch (e: Exception) {
                        Exceptions.handle(e) { DualTelemetry.smartLog(it) }
                    }
                    try {
                        Thread.sleep(max(0.0, (magMillis - (System.currentTimeMillis() - start)).toDouble()).toLong())
                    } catch (e: InterruptedException) {
                        Thread.currentThread().interrupt()
                    }
                }
            } finally {
                Dbg.logd(Threads::class.java, "thread loop task '%(%)' completed.", name, loopTask.hashCode())
            }
            null
        }
        val scheduled = ResultImpl(ThreadPool.getDefault().submit(t)) { loopTask.run(); null }
        tasks[name] = Pair(loopTask.hashCode(), scheduled)
        Dbg.logd(
            Threads::class.java,
            "starting new thread loop task: %(%) %...",
            name,
            loopTask.hashCode(),
            if (magMillis != 0L) "at a $magMillis ms interval " else ""
        )
        return scheduled
    }

    /**
     * Start a new thread task with the given infinite loop task. These tasks are not designed to return any results,
     * although a [Result] will be returned to you if required. This task loops at the maximum speed possible by
     * the CPU.
     *
     * This will auto-end when the task is interrupted.
     *
     * @param name     the unique name of the task to access it later and to log as
     * @param loopTask the infinite loop task to run on the new thread, contains no result as it is an infinite loop
     * @return an unbounded [Result] for the task, note this will never naturally get a result
     */
    @JvmStatic
    fun startLoop(name: String, loopTask: Runnable) = startLoop(name, Units.Seconds.zero(), loopTask)

    /**
     * Stops all thread tasks that are currently running.
     *
     * This will interrupt all threads and remove them from the list of managed tasks,
     * excluding ones that were not scheduled through the Threads class.
     *
     * **IMPORTANT**: This method is automatically called at the end of any OpMode, via a [Hook].
     */
    @JvmStatic
    @Hook(on = Hook.Target.POST_STOP, priority = 4)
    fun stopAll() {
        for ((key, task) in tasks) {
            if (task.second.isDone) continue
            if (task.second.ignoreStopAll) {
                Dbg.log(
                    Threads::class.java,
                    "ignoring stop request for thread task: %(%, ignoreStopAll) ...",
                    key,
                    task.first
                )
                continue
            }
            Dbg.logd(Threads::class.java, "stopping thread task: %(%) ...", key, task.first)
            task.second.cancel(true)
        }
        tasks.clear()
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the name of the task to check, must be managed by Threads or will return false
     * @return true if the task is running, false otherwise
     */
    @JvmStatic
    fun isRunning(task: String) = tasks[task]?.second?.isDone?.not() ?: false

    /**
     * Check if a task is currently running.
     *
     * @param task the task to check, must be managed by Threads or will return false; using Object supertype for compat. between Runnable and Callable
     * @return true if the task is running, false otherwise
     */
    @JvmStatic
    fun isRunning(task: Any?): Boolean {
        val res = AtomicReference<Result<*>?>()
        tasks.forEach { (_, v) -> if (v.first == task.hashCode()) res.set(v.second) }
        return res.get()?.isDone ?: false
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the name of the task to stop, must be managed by Threads
     */
    @JvmStatic
    fun stop(task: String?) {
        val res = tasks[task]
        if (res == null) {
            Dbg.warn(Threads::class.java, "tried to stop a task '%' that is not being managed by Threads.", task)
            return
        }
        res.second.cancel(true)
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the task to stop, must be managed by Threads;
     * using Object supertype for compat. between Runnable and Callable
     */
    @JvmStatic
    fun stop(task: Any?) {
        val res = AtomicReference<Result<*>?>()
        tasks.forEach { (_, v) -> if (v.first == task.hashCode()) res.set(v.second) }
        val t = res.get()
        if (t == null) {
            Dbg.warn(Threads::class.java, "tried to stop a task '%' that is not being managed by Threads.", task)
            return
        }
        t.cancel(true)
    }

    /**
     * Gets a [Result] from the currently managed tasks by the supplied name.
     *
     * **IMPORTANT:** Will return null if there is no task with such a name that is being managed by Threads.
     *
     * The returned [Result] object can be used to then cancel execution or get (and block for) a result.
     * If getting a result that you actually want, storing the [Result] returned by the start methods
     * will use the proper generic type, as this method only returns unbounded types.
     *
     * @param task the name ID of the task to get, must be managed by Threads
     * @return the (unbounded) task [Result], if found
     */
    @JvmStatic
    fun task(task: String?): Result<*>? = tasks.getOrDefault(task, Pair(null, null)).second

    /**
     * Gets a [Result] from the currently managed tasks by the supplied task.
     *
     * **IMPORTANT:** Will return null if there is no task with such a name that is being managed by Threads.
     *
     * The returned [Result] object can be used to then cancel execution or get (and block for) a result.
     * If getting a result that you actually want, storing the [Result] returned by the start methods
     * will use the proper generic type, as this method only returns unbounded types.
     *
     * @param task the task to get, must be managed by Threads, will be null if not found; using Any supertype for compat. between Runnable and Callable
     * @return the (unbounded) task [Result], if found
     */
    @JvmStatic
    fun task(task: Any?): Result<*>? {
        val res = AtomicReference<Result<*>?>()
        tasks.forEach { (_, v) -> if (v.first == task.hashCode()) res.set(v.second) }
        return res.get()
    }
}
