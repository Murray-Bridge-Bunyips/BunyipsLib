package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.WaitTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class WaitTaskTest extends SchedulerTests {
    @Test
    void waitTaskTest() throws InterruptedException {
        WaitTask waitTask = new WaitTask(0.5, Seconds);
        Scheduler.schedule(waitTask);
        Scheduler.update();
        Thread.sleep(100);
        Scheduler.update();
        assertTrue(waitTask.isRunning());
        Thread.sleep(450);
        Scheduler.update();
        assertFalse(waitTask.isRunning());
    }

    @Test
    void timeoutTest() throws InterruptedException {
        AtomicBoolean task1_init = new AtomicBoolean(), task1_periodic = new AtomicBoolean(), task1_int = new AtomicBoolean();
        Task task1 = Task.task()
                .init(() -> task1_init.set(true))
                .periodic(() -> task1_periodic.set(true))
                .onInterrupt(() -> task1_int.set(true))
                .timeout(Seconds.of(0.1));

        Scheduler.schedule(task1);
        Scheduler.update();

        assertTrue(task1_init.get());
        assertTrue(task1_periodic.get());
        assertTrue(task1.isRunning());

        Thread.sleep(150);
        Scheduler.update();

        assertTrue(task1_int.get());
        assertFalse(task1.isRunning());
    }
}