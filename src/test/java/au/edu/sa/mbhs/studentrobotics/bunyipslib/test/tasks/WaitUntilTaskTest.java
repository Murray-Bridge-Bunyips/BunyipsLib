package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class WaitUntilTaskTest extends SchedulerTests {
    @Test
    void waitUntilTest() {
        AtomicBoolean condition = new AtomicBoolean();
        Task command = Task.waitFor(condition::get);
        Scheduler.schedule(command);
        Scheduler.update();
        assertTrue(command.isRunning());
        condition.set(true);
        Scheduler.update();
        assertFalse(command.isRunning());
    }
}