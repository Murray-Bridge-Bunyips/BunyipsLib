package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ConditionalTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class ConditionalTaskTest extends SchedulerTests {
    @Test
    void conditionalTaskTest() {
        Task task1 = Task.task();
        Task task2 = Task.task();
        ConditionalTask conditionalTask = new ConditionalTask(task1, task2, () -> true);
        Scheduler.schedule(conditionalTask);
        Scheduler.update();
        assertTrue(task1.isRunning());
        assertFalse(task2.isRunning());
    }

    @Test
    void conditionalTaskDependencyTest() {
        BunyipsSubsystem system1 = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        BunyipsSubsystem system2 = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        AtomicBoolean task1Int = new AtomicBoolean(false);
        Task task1 = Task.task()
                .onInterrupt(() -> task1Int.set(true))
                .on(system1);
        AtomicBoolean task2Int = new AtomicBoolean(false);
        Task task2 = Task.task()
                .onInterrupt(() -> task2Int.set(true))
                .on(system2);
        ConditionalTask conditionalTask = new ConditionalTask(task1, task2, () -> true);
        Scheduler.schedule(conditionalTask);
        Scheduler.schedule(new Lambda().on(system1).asPriority());
        // BunyipsLib does not require groups be unscheduled when one of their children classes has dependency conflicts
        // This differs from WPILib but is a fundamental structure of the old Task system that doesn't matter too much
        assertFalse(task1.isRunning());
        assertTrue(task1Int.get());
        assertFalse(task2Int.get());
    }
}
