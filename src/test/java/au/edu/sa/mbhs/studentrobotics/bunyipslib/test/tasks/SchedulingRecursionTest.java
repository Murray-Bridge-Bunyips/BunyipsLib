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
        assertFalse(selfCancels.isRunning());
        assertTrue(other.isRunning());
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
        assertFalse(selfCancels.isRunning());
        assertTrue(other.isRunning());
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
        assertFalse(selfCancels.isRunning());
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
        assertFalse(aCancelsB.isRunning());
        assertFalse(bCancelsC.isRunning());
        assertFalse(cCancelsD.isRunning());
        assertFalse(dCancelsAll.isRunning());
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
        assertFalse(aCancelsB.isRunning());
        assertFalse(bIncrementsCounter.isRunning());
    }

//    @Test
//    void scheduleFromEndCancel() {
//        AtomicInteger counter = new AtomicInteger();
//        Subsystem requirement = new SubsystemBase() {
//        };
//        Command other = requirement.runOnce(() -> {
//        });
//        FunctionalCommand selfCancels =
//                new FunctionalCommand(
//                        () -> {
//                        },
//                        () -> {
//                        },
//                        interrupted -> {
//                            counter.incrementAndGet(;
//                            Scheduler.schedule(other);
//                        },
//                        () -> false,
//                        requirement);
//
//        Scheduler.schedule(selfCancels);
//
//        assertDoesNotThrow(() -> Scheduler.cancel(selfCancels));
//        assertEquals(1, counter.get());
//        assertFalse(selfC.isRunning()ncels));
//    }
//
//    @Test
//    void scheduleFromEndInterrupt() {
//        AtomicInteger counter = new AtomicInteger();
//        Subsystem requirement = new SubsystemBase() {
//        };
//        Command other = requirement.runOnce(() -> {
//        });
//        FunctionalCommand selfCancels =
//                new FunctionalCommand(
//                        () -> {
//                        },
//                        () -> {
//                        },
//                        interrupted -> {
//                            counter.incrementAndGet();
//                            Scheduler.schedule(other);
//                        },
//                        () -> false,
//                        requirement);
//
//        Scheduler.schedule(selfCancels);
//
//        assertDoesNotThrow(() -> Scheduler.schedule(other));
//        assertEquals(1, counter.get());
//        assertFalse(selfC.isRunning()ncels));
//        assertTrue(other.isRunning());
//    }
//
//    @Test
//    void scheduleFromEndInterruptAction() {
//        AtomicInteger counter = new AtomicInteger();
//        Subsystem requirement = new Subsystem() {
//        };
//        Command other = requirement.runOnce(() -> {
//        });
//        Command selfCancels = requirement.runOnce(() -> {
//        });
//        Scheduler.onCommandInterrupt(
//                cmd -> {
//                    counter.incrementAndGet();
//                    Scheduler.schedule(other);
//                });
//        Scheduler.schedule(selfCancels);
//
//        assertDoesNotThrow(() -> Scheduler.schedule(other));
//        assertEquals(1, counter.get());
//        assertFalse(selfC.isRunning()ncels));
//        assertTrue(other.isRunning());
//    }
//
//    @Test
//    void scheduleInitializeFromDefaultCommand() {
//        AtomicInteger counter = new AtomicInteger();
//        Subsystem requirement = new SubsystemBase() {
//        };
//        Command other = requirement.runOnce(() -> {
//        }).withInterruptBehavior(interruptionBehavior);
//        FunctionalCommand defaultCommand =
//                new FunctionalCommand(
//                        () -> {
//                            counter.incrementAndGet();
//                            Scheduler.schedule(other);
//                        },
//                        () -> {
//                        },
//                        interrupted -> {
//                        },
//                        () -> false,
//                        requirement);
//
//        Scheduler.setDefaultCommand(requirement, defaultCommand);
//
//        Scheduler.update();
//        Scheduler.update();
//        Scheduler.update();
//        assertEquals(3, counter.get());
//        assertFalse(defau.isRunning()tCommand));
//        assertTrue(other.isRunning());
//    }
//
//    @Test
//    void cancelDefaultCommandFromEnd() {
//        AtomicInteger counter = new AtomicInteger();
//        Subsystem requirement = new Subsystem() {
//        };
//        Command defaultCommand =
//                new FunctionalCommand(
//                        () -> {
//                        },
//                        () -> {
//                        },
//                        interrupted -> counter.incrementAndGet(),
//                        () -> false,
//                        requirement);
//        Command other = requirement.runOnce(() -> {
//        });
//        Command cancelDefaultCommand =
//                new FunctionalCommand(
//                        () -> {
//                        },
//                        () -> {
//                        },
//                        interrupted -> {
//                            counter.incrementAndGet();
//                            Scheduler.schedule(other);
//                        },
//                        () -> false);
//
//        assertDoesNotThrow(
//                () -> {
//                    Scheduler.schedule(cancelDefaultCommand);
//                    Scheduler.setDefaultCommand(requirement, defaultCommand);
//                    Scheduler.update();
//                    Scheduler.cancel(cancelDefaultCommand);
//                });
//        assertEquals(2, counter.get());
//        assertFalse(defau.isRunning()tCommand));
//        assertTrue(other.isRunning());
//    }
}
