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
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.DeadlineTaskGroup;

class DeadlineTaskGroupTest extends SchedulerTests {
    @Test
    void deadlineGroupScheduleTest() {
        AtomicBoolean t1f = new AtomicBoolean();
        AtomicBoolean i1 = new AtomicBoolean(), e1 = new AtomicBoolean(), in1 = new AtomicBoolean();
        AtomicInteger p1 = new AtomicInteger();
        Task task1 = Task.task()
                .init(() -> i1.set(true))
                .periodic(p1::incrementAndGet)
                .onFinish(() -> e1.set(true))
                .onInterrupt(() -> in1.set(true))
                .isFinished(t1f::get);
        AtomicBoolean i2 = new AtomicBoolean(), p2 = new AtomicBoolean(), e2 = new AtomicBoolean(), in2 = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> i2.set(true))
                .periodic(() -> p2.set(true))
                .onFinish(() -> e2.set(true))
                .onInterrupt(() -> in2.set(true))
                .isFinished(() -> true);
        AtomicBoolean i3 = new AtomicBoolean(), e3 = new AtomicBoolean(), in3 = new AtomicBoolean();
        AtomicInteger p3 = new AtomicInteger();
        Task task3 = Task.task()
                .init(() -> i3.set(true))
                .periodic(p3::incrementAndGet)
                .onFinish(() -> e3.set(true))
                .onInterrupt(() -> in3.set(true));
        Task group = new DeadlineTaskGroup(task1, task2, task3);
        Scheduler.schedule(group);
        Scheduler.update();
        assertTrue(group.isActive());
        t1f.set(true);
        Scheduler.update();
        assertFalse(group.isActive());
        assertTrue(i2.get());
        assertTrue(p2.get());
        assertTrue(e2.get());
        assertFalse(in2.get());
        assertTrue(i1.get());
        assertEquals(2, p1.get());
        assertTrue(e1.get());
        assertFalse(in1.get());
        assertTrue(i3.get());
        assertEquals(2, p3.get());
        assertTrue(e3.get());
        assertTrue(in3.get());
    }

    @Test
    void deadlineGroupInterruptTest() {
        AtomicBoolean i1 = new AtomicBoolean(), e1 = new AtomicBoolean(), in1 = new AtomicBoolean();
        AtomicInteger p1 = new AtomicInteger();
        Task task1 = Task.task()
                .init(() -> i1.set(true))
                .periodic(p1::incrementAndGet)
                .onFinish(() -> e1.set(true))
                .onInterrupt(() -> in1.set(true));
        AtomicBoolean i2 = new AtomicBoolean(), p2 = new AtomicBoolean(), e2 = new AtomicBoolean(), in2 = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> i2.set(true))
                .periodic(() -> p2.set(true))
                .onFinish(() -> e2.set(true))
                .onInterrupt(() -> in2.set(true))
                .isFinished(() -> true);
        Task group = new DeadlineTaskGroup(task1, task2);
        Scheduler.schedule(group);
        Scheduler.update();
        Scheduler.update();
        group.finish();
        assertEquals(2, p1.get());
        assertTrue(e1.get());
        assertTrue(in1.get());
        assertTrue(p2.get());
        assertTrue(e1.get());
        assertTrue(in1.get());
        assertFalse(group.isActive());
    }

    @Test
    void deadlineGroupRequirementTest() {
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
        Task group = new DeadlineTaskGroup(task1, task2);
        Scheduler.schedule(group);
        Scheduler.schedule(task3);
        assertTrue(group.isActive()); // BunyipsLib expected behaviour, group continues
        assertTrue(task3.isActive());
    }
}