package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class DefaultTaskTest extends SchedulerTests {
    @Test
    void defaultTaskScheduleTest() {
        BunyipsSubsystem hasDefaultTask = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };

        Task defaultTask = Task.task().on(hasDefaultTask);
        defaultTask.setAsDefaultTask();
        Scheduler.update();

        assertTrue(defaultTask.isActive());
    }

    @Test
    void defaultTaskInterruptResumeTest() {
        BunyipsSubsystem hasDefaultTask = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task defaultTask = Task.task().on(hasDefaultTask);
        Task interrupter = Task.task().on(hasDefaultTask);
        defaultTask.setAsDefaultTask();
        Scheduler.update();
        Scheduler.schedule(interrupter);
        assertFalse(defaultTask.isActive());
        assertTrue(interrupter.isActive());
        interrupter.finish();
        Scheduler.update();
        assertTrue(defaultTask.isActive());
        assertFalse(interrupter.isActive());
    }

    @Test
    void defaultTaskDisableResumeTest() {
        BunyipsSubsystem hasDefaultTask = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        AtomicBoolean intr = new AtomicBoolean(false);
        Task defaultTask = Task.task()
                .onInterrupt(() -> intr.set(true))
                .on(hasDefaultTask);
        defaultTask.setAsDefaultTask();
        Scheduler.update();
        assertTrue(defaultTask.isActive());
        BunyipsSubsystem.disableAll();
        Scheduler.update();
        assertFalse(defaultTask.isActive());
        BunyipsSubsystem.enableAll();
        Scheduler.update();
        assertTrue(defaultTask.isActive());
        assertTrue(intr.get());
    }
}