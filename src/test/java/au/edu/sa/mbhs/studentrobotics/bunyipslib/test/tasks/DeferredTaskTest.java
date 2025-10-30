package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.DeferredTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class DeferredTaskTest extends SchedulerTests {
    @ParameterizedTest
    @ValueSource(booleans = {true, false})
    void deferredFunctionsTest(boolean interrupted) {
        AtomicBoolean init = new AtomicBoolean();
        AtomicBoolean periodic = new AtomicBoolean();
        AtomicInteger loops = new AtomicInteger();
        AtomicBoolean intr = new AtomicBoolean();
        AtomicBoolean nat = new AtomicBoolean();
        Task innerTask = Task.task()
                .init(() -> init.set(true))
                .periodic(() -> periodic.set(true))
                .addIsFinished((f) -> {
                    loops.incrementAndGet();
                    return f;
                })
                .addOnInterrupt(() -> intr.set(true))
                .onFinish(() -> nat.set(true));
        DeferredTask task = new DeferredTask(() -> innerTask);
        task.ensureInit();
        assertNotNull(task.getTask());
        assertTrue(init.get());
        task.execute();
        assertTrue(periodic.get());
        assertFalse(task.isFinished());
        assertEquals(1, loops.get());
        if (interrupted) {
            task.finish();
            assertTrue(nat.get());
            assertTrue(intr.get());
            assertEquals(1, loops.get());
        } else {
            nat.set(true);
            task.execute(); // need to update cond
            assertTrue(nat.get());
            assertFalse(intr.get());
            assertEquals(2, loops.get());
        }
    }

    @Test
    void deferredSupplierOnlyCalledDuringInit() {
        AtomicInteger i = new AtomicInteger();
        Supplier<Task> supplier = () -> {
            i.getAndIncrement();
            return new Lambda();
        };

        DeferredTask task = new DeferredTask(supplier);
        assertEquals(0, i.get());

        Scheduler.schedule(task);
        assertEquals(1, i.get());
        Scheduler.update();

        Scheduler.schedule(task);
        assertEquals(2, i.get());
    }

    // deferredRequirementsTest does not apply as task groups don't carry dependencies

    @Test
    void deferredNullTaskTest() {
        DeferredTask task = new DeferredTask(() -> null);
        assertDoesNotThrow(task::execute);
    }
}