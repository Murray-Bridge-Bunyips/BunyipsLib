package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class RepeatTaskTest extends SchedulerTests {
    @Test
    void callsMethodsCorrectly() {
        var initCounter = new AtomicInteger(0);
        var exeCounter = new AtomicInteger(0);
        var isFinishedCounter = new AtomicInteger(0);
        var endCounter = new AtomicInteger(0);
        var isFinishedHook = new AtomicBoolean(false);

        var task = Task.task()
                .init(initCounter::incrementAndGet)
                .periodic(exeCounter::incrementAndGet)
                .onFinish(endCounter::incrementAndGet)
                .isFinished(() -> {
                    isFinishedCounter.incrementAndGet();
                    return isFinishedHook.get();
                })
                .repeatedly();

        assertEquals(0, initCounter.get());
        assertEquals(0, exeCounter.get());
        assertEquals(0, isFinishedCounter.get());
        assertEquals(0, endCounter.get());

        Scheduler.schedule(task);
        assertEquals(1, initCounter.get());
        assertEquals(0, exeCounter.get());
        assertEquals(0, isFinishedCounter.get());
        assertEquals(0, endCounter.get());

        isFinishedHook.set(false);
        Scheduler.update();
        assertEquals(1, initCounter.get());
        assertEquals(1, exeCounter.get());
        assertEquals(1, isFinishedCounter.get());
        assertEquals(0, endCounter.get());

        isFinishedHook.set(true);
        Scheduler.update();
        assertEquals(1, initCounter.get());
        assertEquals(2, exeCounter.get());
        assertEquals(2, isFinishedCounter.get());
        assertEquals(1, endCounter.get());

        isFinishedHook.set(false);
        Scheduler.update();
        assertEquals(2, initCounter.get());
        assertEquals(3, exeCounter.get());
        assertEquals(3, isFinishedCounter.get());
        assertEquals(1, endCounter.get());

        isFinishedHook.set(true);
        Scheduler.update();
        assertEquals(2, initCounter.get());
        assertEquals(4, exeCounter.get());
        assertEquals(4, isFinishedCounter.get());
        assertEquals(2, endCounter.get());

        task.finish();
        assertEquals(2, initCounter.get());
        assertEquals(4, exeCounter.get());
        assertEquals(4, isFinishedCounter.get());
        assertEquals(2, endCounter.get());
    }
}