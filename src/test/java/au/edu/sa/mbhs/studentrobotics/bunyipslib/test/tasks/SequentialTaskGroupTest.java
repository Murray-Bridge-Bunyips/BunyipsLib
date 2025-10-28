package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.SequentialTaskGroup;

class SequentialTaskGroupTest extends SchedulerTests {
    @Test
    void sequentialGroupScheduleTest() {
        AtomicBoolean task1_init = new AtomicBoolean(), task1_isFinished = new AtomicBoolean(), task1_periodic = new AtomicBoolean(), task1_onFinish = new AtomicBoolean();
        Task task1 = Task.task()
                .init(() -> task1_init.set(true))
                .periodic(() -> task1_periodic.set(true))
                .onFinish(() -> task1_onFinish.set(true))
                .isFinished(task1_isFinished::get);
        AtomicBoolean task2_init = new AtomicBoolean(), task2_isFinished = new AtomicBoolean(), task2_periodic = new AtomicBoolean(), task2_onFinish = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> task2_init.set(true))
                .periodic(() -> task2_periodic.set(true))
                .onFinish(() -> task2_onFinish.set(true))
                .isFinished(task2_isFinished::get);

        Task group = new SequentialTaskGroup(task1, task2);

        Scheduler.schedule(group);

        assertTrue(task1_init.get());
        assertFalse(task2_init.get());

        task1_isFinished.set(true);
        Scheduler.update();

        assertTrue(task1_periodic.get());
        assertTrue(task1_onFinish.get());

        assertTrue(task2_init.get());
        assertFalse(task2_periodic.get());
        assertFalse(task2_onFinish.get());

        task2_isFinished.set(true);
        Scheduler.update();

        assertTrue(task1_periodic.get());
        assertTrue(task1_onFinish.get());
        assertTrue(task2_periodic.get());
        assertTrue(task2_onFinish.get());

        assertFalse(group.isRunning());
    }

    @Test
    void sequentialGroupInterruptTest() {
        AtomicBoolean task1_isFinished = new AtomicBoolean(), task1_periodic = new AtomicBoolean(), task1_onInterrupt = new AtomicBoolean(), task1_onFinish = new AtomicBoolean();
        Task task1 = Task.task()
                .periodic(() -> task1_periodic.set(true))
                .onInterrupt(() -> task1_onInterrupt.set(true))
                .onFinish(() -> task1_onFinish.set(true))
                .isFinished(task1_isFinished::get);
        AtomicBoolean task2_periodic = new AtomicBoolean(), task2_onInterrupt = new AtomicBoolean(), task2_onFinish = new AtomicBoolean();
        Task task2 = Task.task()
                .periodic(() -> task2_periodic.set(true))
                .onInterrupt(() -> task2_onInterrupt.set(true))
                .onFinish(() -> task2_onFinish.set(true));
        AtomicBoolean task3_init = new AtomicBoolean(), task3_periodic = new AtomicBoolean(), task3_onInterrupt = new AtomicBoolean(), task3_onFinish = new AtomicBoolean();
        Task task3 = Task.task()
                .init(() -> task3_init.set(true))
                .periodic(() -> task3_periodic.set(true))
                .onInterrupt(() -> task3_onInterrupt.set(true))
                .onFinish(() -> task3_onFinish.set(true));

        Task group = new SequentialTaskGroup(task1, task2, task3);

        Scheduler.schedule(group);

        task1_isFinished.set(true);
        Scheduler.update();
        group.finish();
        Scheduler.update();

        assertTrue(task1_periodic.get());
        assertFalse(task1_onInterrupt.get());
        assertTrue(task1_onFinish.get());

        assertFalse(task2_periodic.get());
        assertTrue(task2_onInterrupt.get());
        assertTrue(task2_onFinish.get());

        assertFalse(task3_init.get());
        assertFalse(task3_periodic.get());
        assertFalse(task3_onInterrupt.get());
        assertFalse(task3_onFinish.get());

        assertFalse(group.isRunning());
    }

    @Test
    void notScheduledCancelTest() {
        Task task1 = Task.task();
        Task task2 = Task.task();
        Task group = task1.then(task2);
        assertDoesNotThrow(group::finish);
    }

    @Test
    void sequentialGroupRequirementTest() {
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

        Task task1 = Task.task()
                .on(system1);
        Task task2 = Task.task()
                .on(system2);
        Task task3 = Task.task()
                .on(system1)
                .asPriority();

        Task group = new SequentialTaskGroup(task1, task2);

        Scheduler.schedule(group);
        Scheduler.schedule(task3);

        assertTrue(group.isRunning()); // BunyipsLib behaviour
        assertTrue(task3.isRunning());
    }
}