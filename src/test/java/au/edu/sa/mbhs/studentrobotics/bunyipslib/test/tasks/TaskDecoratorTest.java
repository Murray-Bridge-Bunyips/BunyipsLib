package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

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
    void untilBooleanTest() {
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
    void untilOrderBooleanTest() {
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
    void thenTest() {
        AtomicBoolean condition = new AtomicBoolean(false);
        Task task1 = new Lambda();
        Task task2 = new Lambda(() -> condition.set(true));
        Task group = task1.then(task2);
        Scheduler.schedule(group);
        assertFalse(condition.get());
        Scheduler.update();
        assertTrue(condition.get());
        Scheduler.update();
        assertFalse(group.isRunning());
    }

    @Test
    void duringTest() {
        AtomicBoolean finish = new AtomicBoolean(false);
        Task dictator = Task.task().isFinished(finish::get);
        Task endsBefore = new Lambda();
        Task endsAfter = Task.task();
        Task group = dictator.during(endsBefore, endsAfter);
        Scheduler.schedule(group);
        Scheduler.update();
        assertTrue(group.isRunning());
        finish.set(true);
        Scheduler.update();
        assertFalse(group.isRunning());
    }

    @Test
    void duringOrderTest() {
        AtomicBoolean dictatorHasRun = new AtomicBoolean(false);
        AtomicBoolean dictatorWasPolled = new AtomicBoolean(false);
        Task dictator = Task.task().periodic(() -> dictatorHasRun.set(true))
                .isFinished(() -> {
                    dictatorWasPolled.set(true);
                    return true;
                });
        Task other = new Lambda(() -> assertAll(() -> assertTrue(dictatorHasRun.get()), () -> assertTrue(dictatorWasPolled.get())));
        Task group = dictator.during(other);
        Scheduler.schedule(group);
        Scheduler.update();
        assertAll(() -> assertTrue(dictatorHasRun.get()), () -> assertTrue(dictatorWasPolled.get()));
    }

    @Test
    void untilTest() {
        AtomicBoolean finish = new AtomicBoolean(false);
        Task endsBeforeGroup = new Lambda().until(Task.task().isFinished(finish::get));
        Scheduler.schedule(endsBeforeGroup);
        Scheduler.update();
        assertTrue(endsBeforeGroup.isRunning());
        finish.set(true);
        Scheduler.update();
        assertFalse(endsBeforeGroup.isRunning());
        finish.set(false);
        Task endsAfterGroup = Task.task().until(Task.task().isFinished(finish::get));
        Scheduler.schedule(endsAfterGroup);
        Scheduler.update();
        assertTrue(endsAfterGroup.isRunning());
        finish.set(true);
        Scheduler.update();
        assertFalse(endsAfterGroup.isRunning());
    }

    @Test
    void untilOrderTest() {
        AtomicBoolean dictatorHasRun = new AtomicBoolean(false);
        AtomicBoolean dictatorWasPolled = new AtomicBoolean(false);
        Task dictator = Task.task()
                .periodic(() -> dictatorHasRun.set(true))
                .isFinished(() -> {
                    dictatorWasPolled.set(true);
                    return true;
                });
        Task other = new Lambda(() -> assertAll(() -> assertTrue(dictatorHasRun.get()),
                                        () -> assertTrue(dictatorWasPolled.get())));
        Task group = other.until(dictator);
        Scheduler.schedule(group);
        Scheduler.update();
        assertAll(() -> assertTrue(dictatorHasRun.get()), () -> assertTrue(dictatorWasPolled.get()));
    }

    @Test
    void withTest() {
        AtomicBoolean finish = new AtomicBoolean(false);

        Task task1 = Task.task().isFinished(finish::get);
        Task task2 = new Lambda();

        Task group = task1.with(task2);

        Scheduler.schedule(group);
        Scheduler.update();

        assertTrue(group.isRunning());

        finish.set(true);
        Scheduler.update();

        assertFalse(group.isRunning());
    }

    @Test
    void withOrderTest() {
        AtomicBoolean firstHasRun = new AtomicBoolean(false);
        AtomicBoolean firstWasPolled = new AtomicBoolean(false);
        Task task1 = Task.task()
                .periodic(() -> firstHasRun.set(true))
                .isFinished(() -> {
                    firstWasPolled.set(true);
                    return true;
                });
        Task task2 = new Lambda(() ->
                assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get())));
        Task group = task1.with(task2);
        Scheduler.schedule(group);
        Scheduler.update();
        assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
    }

    @Test
    void raceTest() {
        Task task1 = Task.task();
        Task task2 = new Lambda();

        Task group = task1.race(task2);

        Scheduler.schedule(group);
        Scheduler.update();

        assertFalse(group.isRunning());
    }

    @Test
    void raceOrderTest() {
        AtomicBoolean firstHasRun = new AtomicBoolean(false);
        AtomicBoolean firstWasPolled = new AtomicBoolean(false);

        Task task1 = Task.task()
                .periodic(() -> firstHasRun.set(true))
                .isFinished(() -> {
                    firstWasPolled.set(true);
                    return true;
                });
        Task task2 = new Lambda(() -> {
                            assertTrue(firstHasRun.get());
                            assertTrue(firstWasPolled.get());
                        });

        Task group = task1.race(task2);

        Scheduler.schedule(group);
        Scheduler.update();

        assertAll(() -> assertTrue(firstHasRun.get()), () -> assertTrue(firstWasPolled.get()));
    }

    @Test
    void unlessTest() {
        AtomicBoolean hasRun = new AtomicBoolean(false);
        AtomicBoolean unlessCondition = new AtomicBoolean(true);

        Task task = new Lambda(() -> hasRun.set(true)).unless(unlessCondition::get);

        Scheduler.schedule(task);
        Scheduler.update();
        assertFalse(hasRun.get());

        unlessCondition.set(false);
        Scheduler.schedule(task);
        Scheduler.update();
        assertTrue(hasRun.get());
    }

    @Test
    void onlyIfTest() {
        AtomicBoolean hasRun = new AtomicBoolean(false);
        AtomicBoolean onlyIfCondition = new AtomicBoolean(false);

        Task task = new Lambda(() -> hasRun.set(true)).onlyIf(onlyIfCondition::get);

        Scheduler.schedule(task);
        Scheduler.update();
        assertFalse(hasRun.get());

        onlyIfCondition.set(true);
        Scheduler.schedule(task);
        Scheduler.update();
        assertTrue(hasRun.get());
    }

    @Test
    void mutateOnFinishTest() {
        AtomicInteger first = new AtomicInteger(0);
        AtomicInteger second = new AtomicInteger(0);

        Task task = Task.task()
                    .onFinish(first::incrementAndGet)
                    .isFinished(() -> true)
                    .mutate()
                    .addOnFinish(() -> {
                        // to differentiate between "didn't run" and "ran before task's `end()`
                        second.addAndGet(1 + first.get());
                    });

        Scheduler.schedule(task);
        assertEquals(0, first.get());
        assertEquals(0, second.get());
        Scheduler.update();
        assertEquals(1, first.get());
        // if `second == 0`, neither of the lambdas ran.
        // if `second == 1`, the second lambda ran before the first one
        assertEquals(2, second.get());
    }

    // handleInterruptTest() implicitly tests the interrupt=true branch of mutate and interrupt
    @Test
    void handleInterruptTest() {
        AtomicInteger first = new AtomicInteger(0);
        AtomicInteger second = new AtomicInteger(0);

        Task task = Task.task()
                .onInterrupt(first::incrementAndGet)
                .isFinished(() -> true)
                .mutate()
                .addOnInterrupt(() -> {
                    // to differentiate between "didn't run" and "ran before task's `end()`
                    second.addAndGet(1 + first.get());
                });

        Scheduler.schedule(task);
        Scheduler.update();
        assertEquals(0, first.get());
        assertEquals(0, second.get());

        task.finish();
        assertEquals(1, first.get());
        // if `second == 0`, neither of the lambdas ran.
        // if `second == 1`, the second lambda ran before the first one
        assertEquals(2, second.get());
    }

    @Test
    void namedTest() {
        Task task = new Lambda();
        String name = "Named";
        Task named = task.named(name);
        assertEquals(name, named.toString());
    }
}
