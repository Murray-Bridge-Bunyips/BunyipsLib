package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.Assert.assertThrows;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions;

class TaskDependencyTest extends SchedulerTests {
    @Test
    void dependencyInterruptAsPriorityTest() {
        BunyipsSubsystem requirement = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        AtomicBoolean interruptedInit = new AtomicBoolean(false);
        AtomicBoolean interruptedPeriodic = new AtomicBoolean(false);
        AtomicBoolean interruptedInterrupted = new AtomicBoolean(false);
        Task interrupted = Task.task()
                .init(() -> interruptedInit.set(true))
                .periodic(() -> interruptedPeriodic.set(true))
                .onInterrupt(() -> interruptedInterrupted.set(true))
                .on(requirement);
        AtomicBoolean interrupterInit = new AtomicBoolean(false);
        AtomicBoolean interrupterPeriodic = new AtomicBoolean(false);
        Task interrupter = Task.task()
                .init(() -> interrupterInit.set(true))
                .periodic(() -> interrupterPeriodic.set(true))
                .on(requirement)
                .asPriority(); // important
        Scheduler.schedule(interrupted);
        Scheduler.update();
        Scheduler.schedule(interrupter);
        Scheduler.update();
        assertTrue(interruptedInit.get());
        assertTrue(interruptedPeriodic.get());
        assertTrue(interruptedInterrupted.get());
        assertTrue(interrupterInit.get());
        assertTrue(interrupterPeriodic.get());
        assertFalse(interrupted.isActive());
        assertTrue(interrupter.isActive());
    }

    @Test
    void dependencyUninterruptibleTest() {
        BunyipsSubsystem system = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task stoic = Task.task().on(system);
        Task interrupter = Task.task().on(system);
        Scheduler.schedule(stoic);
        Scheduler.schedule(interrupter);
        assertTrue(stoic.isActive());
        assertFalse(interrupter.isActive());
    }

    @Test
    void defaultTaskNeverEndsTest() {
        BunyipsSubsystem system = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task toEnd = Task.task();
        system.setDefaultTask(toEnd);
        system.update();
        toEnd.finish();
        assertThrows(Exceptions.EmergencyStop.class, system::update);
    }
}
