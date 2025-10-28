package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class InitFinishTaskTest extends SchedulerTests {
    @Test
    void dynamicTaskInitFinishScheduleTest() {
        AtomicBoolean cond1 = new AtomicBoolean();
        AtomicBoolean cond2 = new AtomicBoolean();

        Task task = Task.task()
                .init(() -> cond1.set(true))
                .onFinish(() -> cond2.set(true));

        Scheduler.schedule(task);
        Scheduler.update();

        assertTrue(task.isRunning());

        task.finish();

        assertFalse(task.isRunning());
        assertTrue(cond1.get());
        assertTrue(cond2.get());
    }
}