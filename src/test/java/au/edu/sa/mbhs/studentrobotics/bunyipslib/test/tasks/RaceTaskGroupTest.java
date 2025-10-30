package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.groups.RaceTaskGroup;

class RaceTaskGroupTest extends SchedulerTests {
    @Test
    void raceGroupScheduleTest() {
        AtomicBoolean i1 = new AtomicBoolean(), f1 = new AtomicBoolean(), p1 = new AtomicBoolean(), in1 = new AtomicBoolean();
        Task task1 = Task.task()
                .init(() -> i1.set(true))
                .periodic(() -> p1.set(true))
                .isFinished(f1::get)
                .onInterrupt(() -> in1.set(true));
        AtomicBoolean i2 = new AtomicBoolean(), f2 = new AtomicBoolean(), p2 = new AtomicBoolean(), in2 = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> i2.set(true))
                .periodic(() -> p2.set(true))
                .isFinished(f2::get)
                .onInterrupt(() -> in2.set(true));
        Task group = new RaceTaskGroup(task1, task2);
        Scheduler.schedule(group);
        assertTrue(i1.get());
        assertTrue(i2.get());
        f1.set(true);
        Scheduler.update();
        f2.set(true);
        Scheduler.update();
        assertTrue(p1.get());
        assertFalse(in1.get());
        assertTrue(p2.get());
        assertTrue(in2.get());
        assertFalse(group.isActive());
    }

    @Test
    void raceGroupInterruptTest() {
        AtomicInteger l1 = new AtomicInteger();
        AtomicBoolean i1 = new AtomicBoolean();
        Task task1 = Task.task()
                .periodic(l1::incrementAndGet)
                .onInterrupt(() -> i1.set(true));
        AtomicInteger l2 = new AtomicInteger();
        AtomicBoolean i2 = new AtomicBoolean();
        Task task2 = Task.task()
                .periodic(l2::incrementAndGet)
                .onInterrupt(() -> i2.set(true));
        Task group = new RaceTaskGroup(task1, task2);
        Scheduler.schedule(group);
        Scheduler.update();
        Scheduler.update();
        group.finish();
        assertEquals(2, l1.get());
        assertTrue(i1.get());
        assertEquals(2, l2.get());
        assertTrue(i2.get());
        assertFalse(group.isActive());
    }

    @Test
    void raceGroupRequirementTest() {
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
        Task group = new RaceTaskGroup(task1, task2);
        Scheduler.schedule(group);
        Scheduler.schedule(task3);
        assertTrue(group.isActive()); // BunyipsLib expected behaviour, group continues
        assertTrue(task3.isActive());
    }

    @Test
    void raceGroupOnlyCallsEndOnceTest() {
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
        AtomicBoolean t1 = new AtomicBoolean();
        Task task1 = Task.task().isFinished(t1::get).on(system1);
        AtomicBoolean t2 = new AtomicBoolean();
        Task task2 = Task.task().isFinished(t2::get).on(system2);
        Task task3 = Task.task();
        Task group1 = Task.seq(task1, task2);
        assertNotNull(group1);
        assertNotNull(task3);
        Task group2 = Task.rce(group1, task3);
        Scheduler.schedule(group2);
        Scheduler.update();
        t1.set(true);
        Scheduler.update();
        t2.set(true);
        // at this point the sequential group should be done
        assertDoesNotThrow(Scheduler::update);
        assertFalse(group2.isActive());
    }

    @Test
    void raceGroupScheduleTwiceTest() {
        AtomicBoolean t1 = new AtomicBoolean(), t1init = new AtomicBoolean(), t1periodic = new AtomicBoolean();
        Task task1 = Task.task()
                .init(() -> t1init.set(true))
                .periodic(() -> t1periodic.set(true))
                .isFinished(t1::get);
        AtomicBoolean t2 = new AtomicBoolean(), t2init = new AtomicBoolean(), t2periodic = new AtomicBoolean(), t2int = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> t2init.set(true))
                .periodic(() -> t2periodic.set(true))
                .onInterrupt(() -> t2int.set(true))
                .isFinished(t2::get);
        Task group = new RaceTaskGroup(task1, task2);
        Scheduler.schedule(group);
        assertTrue(t1init.get());
        assertTrue(t2init.get());
        t1.set(true);
        Scheduler.update();
        t2.set(true);
        Scheduler.update();
        assertTrue(t1periodic.get());
        assertTrue(task1.isFinished());
        assertTrue(t2periodic.get());
        assertTrue(task2.isFinished());
        assertTrue(t2int.get());
        assertFalse(group.isActive());
        t1.set(false);
        t1init.set(false);
        task1.reset();
        t2.set(false);
        t2init.set(false);
        task2.reset();
        Scheduler.schedule(group);
        assertTrue(t1init.get());
        assertTrue(t2init.get());
        Scheduler.update();
        Scheduler.update();
        assertTrue(group.isActive());
        t2.set(true);
        Scheduler.update();
        assertFalse(group.isActive());
    }
}