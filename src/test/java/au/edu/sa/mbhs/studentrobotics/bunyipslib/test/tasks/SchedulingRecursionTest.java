package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class SchedulingRecursionTest extends SchedulerTests {
    @Test
    void cancelFromInitialize() {
        AtomicBoolean hasOtherRun = new AtomicBoolean();
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task selfCancels = Task.task()
                .init(Task::finish)
                .onFinish(counter::incrementAndGet)
                .on(requirement);
        Task other = Task.loop(() -> hasOtherRun.set(true)).on(requirement);

        assertDoesNotThrow(
                () -> {
                    Scheduler.schedule(selfCancels);
                    Scheduler.update();
                    Scheduler.schedule(other);
                });
        assertFalse(selfCancels.isActive());
        assertTrue(other.isActive());
        assertEquals(1, counter.get());
        Scheduler.update();
        assertTrue(hasOtherRun.get());
    }

    @Test
    void defaultCommandGetsRescheduledAfterSelfCanceling() {
        AtomicBoolean hasOtherRun = new AtomicBoolean();
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task selfCancels = Task.task()
                .init(Task::finish)
                .onFinish(counter::incrementAndGet)
                .on(requirement);
        Task other = Task.loop(() -> hasOtherRun.set(true)).on(requirement);
        other.setAsDefaultTask();

        assertDoesNotThrow(
                () -> {
                    Scheduler.schedule(selfCancels);
                    Scheduler.update();
                });
        Scheduler.update();
        assertFalse(selfCancels.isActive());
        assertTrue(other.isActive());
        assertEquals(1, counter.get());
        Scheduler.update();
        assertTrue(hasOtherRun.get());
    }

    @Test
    void cancelFromEnd() {
        AtomicInteger counter = new AtomicInteger();
        Task selfCancels = Task.task()
                .onFinish((t) -> {
                    counter.incrementAndGet();
                    t.finish();
                });
        Scheduler.schedule(selfCancels);

        assertDoesNotThrow(selfCancels::finish);
        assertEquals(1, counter.get());
        assertFalse(selfCancels.isActive());
    }

    @Test
    void cancelFromEndLoop() {
        AtomicInteger counter = new AtomicInteger();
        Task dCancelsAll = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    Scheduler.finishAll();
                })
                .isFinished(() -> true);
        Task cCancelsD = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    dCancelsAll.finish();
                })
                .isFinished(() -> true);
        Task bCancelsC = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    cCancelsD.finish();
                })
                .isFinished(() -> true);
        Task aCancelsB = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    bCancelsC.finish();
                })
                .isFinished(() -> true);

        Scheduler.schedule(aCancelsB);
        Scheduler.schedule(bCancelsC);
        Scheduler.schedule(cCancelsD);
        Scheduler.schedule(dCancelsAll);

        assertDoesNotThrow(aCancelsB::finish);
        assertEquals(4, counter.get());
        assertFalse(aCancelsB.isActive());
        assertFalse(bCancelsC.isActive());
        assertFalse(cCancelsD.isActive());
        assertFalse(dCancelsAll.isActive());
    }

    @Test
    void multiCancelFromEnd() {
        AtomicInteger counter = new AtomicInteger();
        Task bIncrementsCounter = Task.task().onFinish(counter::incrementAndGet).isFinished(() -> true);
        Task aCancelsB = Task.task()
                .onFinish((t) -> {
                    counter.incrementAndGet();
                    bIncrementsCounter.finish();
                    t.finish();
                });

        Scheduler.schedule(aCancelsB);
        Scheduler.schedule(bIncrementsCounter);

        assertDoesNotThrow(aCancelsB::finish);
        assertEquals(2, counter.get());
        assertFalse(aCancelsB.isActive());
        assertFalse(bIncrementsCounter.isActive());
    }

    @Test
    void scheduleFromEndCancel() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task other = new Lambda().on(requirement);
        Task selfCancels = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    Scheduler.schedule(other);
                })
                .on(requirement);

        Scheduler.schedule(selfCancels);

        assertDoesNotThrow(selfCancels::finish);
        assertEquals(1, counter.get());
        assertFalse(selfCancels.isActive());
    }

    @Test
    void scheduleFromEndInterrupt() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task other = new Lambda().on(requirement).asPriority();
        Task selfCancels = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    Scheduler.schedule(other);
                })
                .on(requirement);

        Scheduler.schedule(selfCancels);

        assertDoesNotThrow(() -> Scheduler.schedule(other));
        assertEquals(1, counter.get());
        assertFalse(selfCancels.isActive());
        assertTrue(other.isActive());
    }

    @Test
    void scheduleFromEndInterruptAction() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task other = new Lambda()
                .on(requirement)
                .asPriority();
        Task selfCancels = new Lambda()
                .mutate()
                .onInterrupt(() -> {
                    counter.incrementAndGet();
                    Scheduler.schedule(other);
                })
                .on(requirement);
        Scheduler.schedule(selfCancels);

        assertDoesNotThrow(() -> Scheduler.schedule(other));
        assertEquals(1, counter.get());
        assertFalse(selfCancels.isActive());
        assertTrue(other.isActive());
    }

    @Test
    void scheduleInitializeFromDefaultCommand() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Scheduler.use(requirement);
        Task other = new Lambda().on(requirement).named("Other");
        Task defaultTask = Task.task()
                .init(() -> {
                    counter.incrementAndGet();
                    Scheduler.schedule(other);
                })
                .on(requirement)
                .named("Default Task");
        requirement.setDefaultTask(defaultTask);

        // BunyipsLib expects that tasks on subsystems are executed/swapped once per cycle and can switch once per cycle.
        // This differs from WPILib where we would expect counter to increment on each update. This also means we can
        // have two tasks "active" at the same time for default and rescheduled as ensureInit fires.
        Scheduler.update(); // init default
        Scheduler.update(); // execute other
        Scheduler.update(); // init default
        Scheduler.update(); // execute other
        Scheduler.update(); // init default
        assertEquals(3, counter.get());
        assertTrue(defaultTask.isActive());
        assertTrue(other.isActive()); // other has been scheduled by init of defaultTask, it is ensureInited but not executing
    }

    @Test
    void cancelDefaultCommandFromEnd() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task defaultTask = Task.task()
                .onFinish(counter::incrementAndGet)
                .on(requirement);
        Task other = new Lambda().on(requirement);
        Task cancelDefaultTask = Task.task()
                .onFinish(() -> {
                    counter.incrementAndGet();
                    Scheduler.schedule(other);
                });

        assertDoesNotThrow(
                () -> {
                    Scheduler.schedule(cancelDefaultTask);
                    requirement.setDefaultTask(defaultTask);
                    Scheduler.update();
                    cancelDefaultTask.finish();
                });
        assertEquals(2, counter.get());
        assertFalse(defaultTask.isActive());
        assertTrue(other.isActive());
    }
}
