package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.ParallelTaskGroup;

class ParallelTaskGroupTest extends SchedulerTests {
    @Test
    void parallelGroupScheduleTest() {
        AtomicBoolean task1Init = new AtomicBoolean();
        AtomicBoolean task1ShouldFinish = new AtomicBoolean();
        AtomicInteger task1Loops = new AtomicInteger();
        Task task1 = Task.task()
                .init(() -> task1Init.set(true))
                .periodic(task1Loops::incrementAndGet)
                .isFinished(task1ShouldFinish::get);
        AtomicBoolean task2Init = new AtomicBoolean();
        AtomicBoolean task2ShouldFinish = new AtomicBoolean();
        AtomicInteger task2Loops = new AtomicInteger();
        Task task2 = Task.task()
                .init(() -> task2Init.set(true))
                .periodic(task2Loops::incrementAndGet)
                .isFinished(task2ShouldFinish::get);
        Task group = new ParallelTaskGroup(task1, task2);
        Scheduler.schedule(group);
        assertTrue(task1Init.get());
        assertTrue(task2Init.get());
        task1ShouldFinish.set(true);
        Scheduler.update();
        task2ShouldFinish.set(true);
        Scheduler.update();
        assertEquals(1, task1Loops.get());
        assertTrue(task1.isFinished());
        assertEquals(2, task2Loops.get());
        assertTrue(task2.isFinished());
        assertFalse(group.isRunning());
    }

    @Test
    void parallelGroupInterruptTest() {
        AtomicInteger t1periodic = new AtomicInteger();
        AtomicBoolean t1finished = new AtomicBoolean();
        AtomicBoolean t1interrupted = new AtomicBoolean();
        AtomicBoolean t1cond = new AtomicBoolean();
        Task task1 = Task.task()
                .periodic(t1periodic::incrementAndGet)
                .isFinished(t1cond::get)
                .onFinish(() -> t1finished.set(true))
                .onInterrupt(() -> t1interrupted.set(true));
        AtomicInteger t2periodic = new AtomicInteger();
        AtomicBoolean t2finished = new AtomicBoolean();
        AtomicBoolean t2interrupted = new AtomicBoolean();
        Task task2 = Task.task()
                .periodic(t2periodic::incrementAndGet)
                .onFinish(() -> t2finished.set(true))
                .onInterrupt(() -> t2interrupted.set(true));
        Task group = task1.with(task2);
        Scheduler.schedule(group);
        t1cond.set(true);
        Scheduler.update();
        Scheduler.update();
        group.finish();
        assertEquals(1, t1periodic.get());
        assertTrue(t1finished.get());
        assertFalse(t1interrupted.get());
        assertEquals(2, t2periodic.get());
        assertTrue(t2finished.get());
        assertTrue(t2interrupted.get());
        assertFalse(group.isRunning());
    }

    @Test
    void parallelGroupRequirementTest() {
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
        Task task1 = Task.task().on(system1);
        Task task2 = Task.task().on(system2);
        Task task3 = Task.task().on(system2).asPriority();
        Task group = new ParallelTaskGroup(task1, task2);
        Scheduler.schedule(group);
        Scheduler.schedule(task3);
        // Note: BunyipsLib does not cancel groups on dependency override, hence this does not match WPILib.
        // Users must instead be careful about their use of groups with subsystem dependencies, and appropriately
        // ensure overwriting tasks are set with correct end conditions and priorities
        assertTrue(group.isRunning());
        assertTrue(task3.isRunning());
    }
}