package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class PeriodicTaskTest extends SchedulerTests {
    @Test
    void loopTaskScheduleTest() {
        AtomicInteger counter = new AtomicInteger(0);
        Task task = Task.loop(counter::incrementAndGet);
        Scheduler.schedule(task);
        Scheduler.update();
        Scheduler.update();
        Scheduler.update();
        assertEquals(3, counter.get());
    }
}