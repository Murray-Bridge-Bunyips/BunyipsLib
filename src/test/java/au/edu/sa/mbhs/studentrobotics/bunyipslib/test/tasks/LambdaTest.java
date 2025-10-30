package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;

class LambdaTest extends SchedulerTests {
    @Test
    void lambdaTaskScheduleTest() {
        AtomicBoolean cond = new AtomicBoolean();
        Lambda task = new Lambda(() -> cond.set(true));
        Scheduler.schedule(task);
        Scheduler.update();
        assertTrue(cond.get());
        assertFalse(task.isActive());
    }
}