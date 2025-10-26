package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class TaskScheduleTest extends SchedulerTests {
    @Test
    void instantScheduleTest() {
        AtomicBoolean init = new AtomicBoolean(false);
        AtomicBoolean periodic = new AtomicBoolean(false);
        AtomicBoolean onFinish = new AtomicBoolean(false);
        Task task = Task.task()
                .init(() -> init.set(true))
                .periodic(() -> periodic.set(true))
                .onFinish(() -> onFinish.set(true))
                .isFinished(() -> true);
        Scheduler.schedule(task);
        assertTrue(task.isRunning());
        assertTrue(init.get());
        Scheduler.update();
        assertTrue(periodic.get());
        assertTrue(onFinish.get());
        assertFalse(task.isRunning());
    }

    @Test
    void singleIterationScheduleTest() {
        AtomicBoolean cond = new AtomicBoolean(false);
        AtomicInteger loops = new AtomicInteger(0);
        AtomicBoolean init = new AtomicBoolean(false);
        AtomicBoolean onFinish = new AtomicBoolean(false);
        AtomicBoolean onInterrupt = new AtomicBoolean(false);
        Task task = Task.task()
                .init(() -> init.set(true))
                .onFinish(() -> onFinish.set(true))
                .onInterrupt(() -> onInterrupt.set(true))
                .periodic(loops::incrementAndGet)
                .isFinished(cond::get);
        Scheduler.schedule(task);
        assertTrue(task.isRunning());
        Scheduler.update();
        cond.set(true);
        Scheduler.update();
        assertTrue(init.get());
        assertEquals(2, loops.get());
        assertTrue(onFinish.get());
        assertFalse(onInterrupt.get());
        assertFalse(task.isRunning());
    }

    @Test
    void multiScheduleTest() {
        Task task1 = Task.task();
        Task task2 = Task.task();
        Task task3 = Task.task();
        Scheduler.schedule(task1);
        Scheduler.schedule(task2);
        Scheduler.schedule(task3);
        assertTrue(task1.isRunning());
        assertTrue(task2.isRunning());
        assertTrue(task3.isRunning());
        Scheduler.update();
        assertTrue(task1.isRunning());
        assertTrue(task2.isRunning());
        assertTrue(task3.isRunning());
        task1.finish();
        Scheduler.update();
        assertTrue(task2.isRunning());
        assertTrue(task3.isRunning());
        assertFalse(task1.isRunning());
        task2.finish();
        Scheduler.update();
        assertTrue(task3.isRunning());
        assertFalse(task1.isRunning());
        assertFalse(task2.isRunning());
        task3.finish();
        Scheduler.update();
        assertFalse(task3.isRunning());
        assertFalse(task1.isRunning());
        assertFalse(task2.isRunning());
    }

    @Test
    void schedulerFinishTest() {
        AtomicBoolean fin = new AtomicBoolean();
        AtomicBoolean inter = new AtomicBoolean();
        Task task = Task.task()
                .onFinish(() -> fin.set(true))
                .onInterrupt(() -> inter.set(true));
        Scheduler.schedule(task);
        Scheduler.update();
        Scheduler.activeTasks.forEach(Task::finish);
        Scheduler.update();
        assertTrue(fin.get());
        assertTrue(inter.get());
        assertFalse(task.isRunning());
    }

    @Test
    void notScheduledTest() {
        Task task = Task.task();
        assertFalse(() -> Scheduler.activeTasks.contains(task));
    }
}