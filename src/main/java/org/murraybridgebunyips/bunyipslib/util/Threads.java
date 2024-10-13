package org.murraybridgebunyips.bunyipslib.util;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Exceptions;
import org.murraybridgebunyips.bunyipslib.While;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

import java.util.HashMap;

/**
 * Async utilities for running user code on different threads while having control/logging over them.
 *
 * @author Lucas Bubner, 2024
 * @see While
 * @since 1.0.0-pre
 */
public final class Threads {
    private static final HashMap<Integer, Thread> threads = new HashMap<>();
    private static final Thread.UncaughtExceptionHandler exceptionHandler = (t, e) ->
            Exceptions.handle(e, BunyipsOpMode.isRunning() ? BunyipsOpMode.getInstance().telemetry::log : null);

    private Threads() {
    }

    /**
     * Start a new thread with the given task.
     *
     * @param task the runnable task to run on the new thread
     * @param name the name of the thread to access it later and to log as
     */
    public static void start(Runnable task, String name) {
        Thread thread = new Thread(() -> {
            try {
                task.run();
            } finally {
                Dbg.logd(Threads.class, "thread '%(%)' completed.", name, Thread.currentThread().hashCode());
            }
        });
        thread.setName(name);
        thread.setUncaughtExceptionHandler(exceptionHandler);
        Dbg.logd(Threads.class, "starting new thread: %(%) ...", name, thread.hashCode());
        thread.start();
        threads.put(task.hashCode(), thread);
    }

    /**
     * Start a new thread with the given task. The thread name will be defined by the class of the Runnable.
     *
     * @param task the runnable task to run on the new thread
     */
    public static void start(Runnable task) {
        start(task, task.getClass().getSimpleName());
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted.
     *
     * @param task              the infinite loop task to run on the new thread
     * @param name              the name of the thread to access it later and to log as
     * @param loopSleepDuration the duration to sleep this thread after every loop to save resources
     */
    public static void startLoop(Runnable task, String name, Measure<Time> loopSleepDuration) {
        long magMillis = (long) Math.abs(loopSleepDuration.in(Milliseconds));
        Thread thread = new Thread(() -> {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    task.run();
                    try {
                        //noinspection BusyWait
                        Thread.sleep(magMillis);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            } finally {
                Dbg.logd(Threads.class, "thread '%(%)' completed.", name, Thread.currentThread().hashCode());
            }
        });
        thread.setName(name);
        thread.setUncaughtExceptionHandler(exceptionHandler);
        Dbg.logd(Threads.class, "starting new loop thread: %(%) %...", name, thread.hashCode(), magMillis != 0 ? "at a " + magMillis + " ms interval " : "");
        thread.start();
        threads.put(task.hashCode(), thread);
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted.
     *
     * @param task the infinite loop task to run on the new thread
     * @param name the name of the thread to access it later and to log as
     */
    public static void startLoop(Runnable task, String name) {
        startLoop(task, name, Seconds.zero());
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted, with a name defined by the class of the Runnable.
     *
     * @param task              the infinite loop task to run on the new thread
     * @param loopSleepDuration the duration to sleep this thread after every loop to save resources
     */
    public static void startLoop(Runnable task, Measure<Time> loopSleepDuration) {
        startLoop(task, task.getClass().getSimpleName(), loopSleepDuration);
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted, with a name defined by the class of the Runnable.
     *
     * @param task the infinite loop task to run on the new thread
     */
    public static void startLoop(Runnable task) {
        startLoop(task, task.getClass().getSimpleName(), Seconds.zero());
    }

    /**
     * Stop all threads that are currently running.
     * This will interrupt all threads and remove them from the list of managed threads,
     * not including threads that were started outside of this class.
     * This method is automatically called at the end of a BunyipsOpMode.
     */
    public static void stopAll() {
        for (Thread thread : threads.values()) {
            if (!thread.isAlive()) continue;
            Dbg.logd(Threads.class, "stopping thread: %(%) ...", thread.getName(), thread.hashCode());
            thread.interrupt();
        }
        threads.clear();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the task to check, must be managed by Threads
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(Runnable task) {
        Thread thread = threads.get(task.hashCode());
        return thread != null && thread.isAlive();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the name of the task to check, must be managed by Threads
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(String task) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                return thread.isAlive();
            }
        }
        return false;
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the task to stop, must be managed by Threads
     */
    public static void stop(Runnable task) {
        Thread thread = threads.get(task.hashCode());
        if (thread != null) {
            if (!thread.isAlive()) return;
            Dbg.logd(Threads.class, "stopping thread: %(%) ...", thread.getName(), thread.hashCode());
            thread.interrupt();
            threads.remove(task.hashCode());
        } else {
            Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task.getClass().getSimpleName());
        }
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the name of the task to stop, must be managed by Threads
     */
    public static void stop(String task) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                if (!thread.isAlive()) return;
                Dbg.logd(Threads.class, "stopping thread: %(%) ...", thread.getName(), thread.hashCode());
                thread.interrupt();
                threads.remove(thread.hashCode());
                return;
            }
        }
        Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task);
    }

    /**
     * Restart a specific task by stopping it and then starting it again.
     *
     * @param task the task to restart, must be managed by Threads
     */
    public static void restart(Runnable task) {
        Dbg.logd(Threads.class, "attempting to restart task: % ...", task.getClass().getSimpleName());
        stop(task);
        start(task);
    }

    /**
     * Restart a specific task by stopping it and then starting it again.
     *
     * @param task the name of the task to restart, must be managed by Threads
     */
    public static void restart(String task) {
        Dbg.logd(Threads.class, "attempting to restart task: % ...", task);
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                stop(thread.getName());
                start(thread, thread.getName());
                return;
            }
        }
    }

    /**
     * Wait for a specific task to finish running.
     *
     * @param task the task to wait for, must be managed by Threads
     */
    public static void waitFor(Runnable task) {
        waitFor(task, false);
    }

    /**
     * Wait for a specific task to finish running.
     *
     * @param task the name of the task to wait for, must be managed by Threads
     */
    public static void waitFor(String task) {
        waitFor(task, false);
    }

    /**
     * Wait for a specific task to finish running, with the option to interrupt it.
     *
     * @param task      the name of the task to wait for, must be managed by Threads
     * @param interrupt whether to interrupt the task first then wait
     */
    public static void waitFor(String task, boolean interrupt) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                waitFor(thread, interrupt);
                return;
            }
        }
        Dbg.warn(Threads.class, "tried to wait for a task '%' that is not being managed by Threads.", task);
    }

    /**
     * Wait for a specific task to finish running, with the option to interrupt it.
     *
     * @param task      the task to wait for, must be managed by Threads
     * @param interrupt whether to interrupt the task first then wait
     */
    public static void waitFor(Runnable task, boolean interrupt) {
        Thread thread = threads.get(task.hashCode());
        if (thread != null) {
            if (!thread.isAlive()) return;
            if (interrupt) {
                Dbg.logd(Threads.class, "stopping thread: %(%) ...", thread.getName(), thread.hashCode());
                thread.interrupt();
            }
            try {
                Dbg.logd(Threads.class, "waiting for thread: %(%) ...", thread.getName(), thread.hashCode());
                thread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        } else {
            Dbg.warn(Threads.class, "tried to wait for a task '%' that is not being managed by Threads.", task.getClass().getSimpleName());
        }
    }
}
