package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class DynamicTaskTest extends SchedulerTests {
    @Test
    void dynamicTaskScheduleTest() {
        AtomicBoolean cond1 = new AtomicBoolean();
        AtomicBoolean cond2 = new AtomicBoolean();
        AtomicBoolean cond3 = new AtomicBoolean();
        AtomicBoolean cond4 = new AtomicBoolean();
        Task task = Task.task()
                .init(() -> cond1.set(true))
                .periodic(() -> cond2.set(true))
                .onFinish(() -> cond3.set(true))
                .isFinished(cond4::get);
        Scheduler.schedule(task);
        Scheduler.update();
        assertTrue(task.isRunning());
        cond4.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
        assertTrue(cond1.get());
        assertTrue(cond2.get());
        assertTrue(cond3.get());
    }
}