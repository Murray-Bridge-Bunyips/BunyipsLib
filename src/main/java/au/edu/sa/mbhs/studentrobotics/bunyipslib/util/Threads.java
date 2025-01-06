package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.ThreadPool;

import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Exceptions;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks.Hook;

/**
 * Async utilities for running user code on different threads while having control/logging over them.
 * <p>
 * <b>WARNING:</b> Multithreading is a highly advanced topic where issues such as race conditions can arise.
 * You must be fully aware of the consequences of threading and whether it is necessary for your program. For 99% of use
 * cases, threading is not required for normal robot operation. Also note that there exists a global hardware lock
 * for the SDK, so attempting to multi-thread hardware for loop times will drastically hinder robot performance.
 * Threads are best used for blocking operations that do not access hardware reads and writes.
 * <p>
 * Tasks are scheduled through the SDK's default cached {@link ThreadPool}.
 * Exceptions thrown during a thread task are caught by the {@link Exceptions} handler.
 * <p>
 * Threads started via this class are automatically shut down at the end of OpModes, via the {@link #stopAll()} method
 * and a BunyipsLib {@link Hook}.
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
public final class Threads {
    private static final HashMap<String, Pair<Integer, Future<?>>> tasks = new HashMap<>();

    private Threads() {
        throw new AssertionError("This is a utility class");
    }

    /**
     * Start a new thread task with the given {@link Callable} that may return a result.
     * <p>
     * A result does not need to be utilised (as it can be ignored), but one must be supplied via the
     * {@link Callable} interface. You can choose to return a null value if there is no results you need.
     * <p>
     * For tasks that run repeatedly, consider {@link #startLoop(String, Measure, Runnable)}, which takes
     * in a {@link Runnable}.
     *
     * @param <T>  return type for the returned {@link Future}
     * @param name the unique name of the task to access it later and to log as
     * @param task the runnable task to run on the new thread
     * @return a {@link Future} of type T that can be used to retrieve the result of this task
     */
    public static <T> Future<T> start(@NonNull String name, @NonNull Callable<T> task) {
        if (tasks.keySet().stream().anyMatch(s -> s.equals(name)))
            throw new IllegalArgumentException("name parameter '" + name + "' is already being used as a Threads task name; it must be unique");
        Callable<T> t = () -> {
            try {
                return task.call();
            } catch (Exception e) {
                Exceptions.handle(e, DualTelemetry::smartLog);
            } finally {
                Dbg.logd(Threads.class, "thread task '%(%)' completed.", name, task.hashCode());
            }
            return (T) null;
        };
        Future<T> scheduled = ThreadPool.getDefault().submit(task);
        tasks.put(name, new Pair<>(task.hashCode(), scheduled));
        Dbg.logd(Threads.class, "starting new thread task: %(%) ...", name, task.hashCode());
        return scheduled;
    }

    /**
     * Start a new thread task with the given infinite loop task. These tasks are not designed to return any results,
     * although a {@link Future} will be returned to you if required.
     * <p>
     * This will auto-end when the task is interrupted.
     *
     * @param name        the unique name of the task to access it later and to log as
     * @param minLoopTime the minimum duration this loop should run at to save resources, will be slower if your loop takes longer than the minimum
     * @param loopTask    the infinite loop task to run on the new thread, contains no result as it is an infinite loop
     * @return an unbounded {@link Future} for the task, note this will never naturally get a result
     */
    public static Future<?> startLoop(@NonNull String name, @NonNull Measure<Time> minLoopTime, @NonNull Runnable loopTask) {
        if (tasks.keySet().stream().anyMatch(s -> s.equals(name)))
            throw new IllegalArgumentException("name parameter '" + name + "' is already being used as a Threads task name; it must be unique");
        long magMillis = (long) minLoopTime.in(Milliseconds);
        Runnable t = () -> {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    long start = System.currentTimeMillis();
                    try {
                        loopTask.run();
                    } catch (Exception e) {
                        Exceptions.handle(e, DualTelemetry::smartLog);
                    }
                    try {
                        //noinspection BusyWait
                        Thread.sleep(Math.max(0, magMillis - (System.currentTimeMillis() - start)));
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            } finally {
                Dbg.logd(Threads.class, "thread loop task '%(%)' completed.", name, loopTask.hashCode());
            }
        };
        Future<?> scheduled = ThreadPool.getDefault().submit(loopTask);
        tasks.put(name, new Pair<>(loopTask.hashCode(), scheduled));
        Dbg.logd(Threads.class, "starting new thread loop task: %(%) %...", name, loopTask.hashCode(), magMillis != 0 ? "at a " + magMillis + " ms interval " : "");
        return scheduled;
    }

    /**
     * Start a new thread task with the given infinite loop task. These tasks are not designed to return any results,
     * although a {@link Future} will be returned to you if required. This task loops at the maximum speed possible by
     * the CPU.
     * <p>
     * This will auto-end when the task is interrupted.
     *
     * @param name     the unique name of the task to access it later and to log as
     * @param loopTask the infinite loop task to run on the new thread, contains no result as it is an infinite loop
     * @return an unbounded {@link Future} for the task, note this will never naturally get a result
     */
    public static Future<?> startLoop(@NonNull String name, @NonNull Runnable loopTask) {
        return startLoop(name, Seconds.zero(), loopTask);
    }

    /**
     * Stops all thread tasks that are currently running.
     * <p>
     * This will interrupt all threads and remove them from the list of managed tasks,
     * excluding ones that were not scheduled through the Threads class.
     * <p>
     * <b>IMPORTANT</b>: This method is automatically called at the end of any OpMode, via a {@link Hook}.
     */
    @Hook(on = Hook.Target.POST_STOP, priority = 1)
    public static void stopAll() {
        for (String key : tasks.keySet()) {
            Pair<Integer, Future<?>> task = tasks.get(key);
            if (task == null || task.second.isDone())
                continue;
            Dbg.logd(Threads.class, "stopping thread task: %(%) ...", key, task.first);
            task.second.cancel(true);
        }
        tasks.clear();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the name of the task to check, must be managed by Threads or will return false
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(@NonNull String task) {
        Pair<Integer, Future<?>> res = tasks.get(task);
        return res != null && !res.second.isDone();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the task to check, must be managed by Threads or will return false;
     *             using Object supertype for compat. between Runnable and Callable
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(@NonNull Object task) {
        AtomicReference<Future<?>> res = new AtomicReference<>();
        tasks.forEach((k, v) -> {
            if (v.first == task.hashCode())
                res.set(v.second);
        });
        return res.get() != null && !res.get().isDone();
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the name of the task to stop, must be managed by Threads
     */
    public static void stop(@Nullable String task) {
        Pair<Integer, Future<?>> res = tasks.get(task);
        if (res == null) {
            Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task);
            return;
        }
        res.second.cancel(true);
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the task to stop, must be managed by Threads;
     *             using Object supertype for compat. between Runnable and Callable
     */
    public static void stop(@NonNull Object task) {
        AtomicReference<Future<?>> res = new AtomicReference<>();
        tasks.forEach((k, v) -> {
            if (v.first == task.hashCode())
                res.set(v.second);
        });
        if (res.get() == null) {
            Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task);
            return;
        }
        res.get().cancel(true);
    }

    /**
     * Gets a {@link Future} from the currently managed tasks by the supplied name.
     * <p>
     * <b>IMPORTANT:</b> Will return null if there is no task with such a name that is being managed by Threads.
     * <p>
     * The returned {@link Future} object can be used to then cancel execution or get (and block for) a result.
     * If getting a result that you actually want, storing the {@link Future} returned by the start methods
     * will use the proper generic type, as this method only returns unbounded types.
     *
     * @param task the name ID of the task to get
     * @return the (unbounded) task {@link Future}, if found
     */
    public static Future<?> task(@NonNull String task) {
        return Objects.requireNonNull(tasks.getOrDefault(task, new Pair<>(-1, null))).second;
    }
}
