package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class TaskDecoratorTest extends SchedulerTests {
    @Test
    void withTimeoutTest() throws InterruptedException {
        Task timeout = Task.task().timeout(Seconds.of(0.1));
        Scheduler.schedule(timeout);
        Scheduler.update();
        assertTrue(timeout.isRunning());
        Thread.sleep(150);
        Scheduler.update();
        assertFalse(timeout.isRunning());
    }

    @Test
    void untilTest() {
        AtomicBoolean finish = new AtomicBoolean();
        Task task = Task.task().until(finish::get);
        Scheduler.schedule(task);
        Scheduler.update();
        assertTrue(task.isRunning());
        finish.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    @Test
    void untilOrderTest() {
        AtomicBoolean firstHasRun = new AtomicBoolean(false);
        AtomicBoolean firstWasPolled = new AtomicBoolean(false);

        Task first = Task.task()
                .periodic(() -> firstHasRun.set(true))
                .isFinished(() -> {
                    firstWasPolled.set(true);
                    return true;
                });
        Task second =
                first.until(
                        () -> {
                            assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
                            return true;
                        });

        Scheduler.schedule(second);
        Scheduler.update();

        assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
    }

    @Test
    void onlyWhileTest() {
        AtomicBoolean run = new AtomicBoolean(true);

        Task task = Task.task().onlyWhile(run::get);

        Scheduler.schedule(task);
        Scheduler.update();

        assertTrue(task.isRunning());

        run.set(false);
        Scheduler.update();

        assertFalse(task.isRunning());
    }

    @Test
    void onlyWhileOrderTest() {
        AtomicBoolean firstHasRun = new AtomicBoolean(false);
        AtomicBoolean firstWasPolled = new AtomicBoolean(false);

        Task first = Task.task()
                .periodic(() -> firstHasRun.set(true))
                .isFinished(() -> {
                    firstWasPolled.set(true);
                    return true;
                });
        Task second =
                first.onlyWhile(
                        () -> {
                            assertAll(
                                    () -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
                            return false;
                        });

        Scheduler.schedule(second);
        Scheduler.update();

        assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
    }

    @Test
    void afterTest() {
        AtomicBoolean finished = new AtomicBoolean();
        finished.set(false);
        Task task = new Lambda().after(() -> finished.set(true));
        Scheduler.schedule(task);
        assertTrue(finished.get());
        Scheduler.update();
        assertTrue(task.isRunning());
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    @Test
    void thenLambdaTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = new Lambda().then(() -> finished.set(true));
        Scheduler.schedule(task);
        assertFalse(finished.get());
        Scheduler.update();
        assertTrue(finished.get());
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    @Test
    void andThenTest() {
        AtomicBoolean condition = new AtomicBoolean(false);
        Task command1 = new Lambda();
        Task command2 = new Lambda(() -> condition.set(true));
        Task group = command1.then(command2);
        Scheduler.schedule(group);
        assertFalse(condition.get());
        Scheduler.update();
        assertTrue(condition.get());
        Scheduler.update();
        assertFalse(group.isRunning());
    }

    // TODO: up to deadlineForTest
}
