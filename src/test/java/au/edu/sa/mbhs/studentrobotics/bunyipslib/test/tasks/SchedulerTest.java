package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicInteger;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class SchedulerTest extends SchedulerTests {
    @Test
    void schedulerMutateTest() {
        AtomicInteger counter = new AtomicInteger();
        Scheduler.schedule(new Lambda(counter::incrementAndGet)
                .mutate()
                .addInit(counter::incrementAndGet) // important to add init so lambda calls twice
                .periodic(counter::incrementAndGet)
                .onFinish(counter::incrementAndGet));
        Scheduler.update();
        assertEquals(4, counter.get());
    }

    @Test
    void schedulerInterruptIdleTest() {
        AtomicInteger counter = new AtomicInteger();
        Task task = new IdleTask()
                .mutate()
                .onInterrupt(counter::incrementAndGet);
        Scheduler.schedule(task);
        task.finish();
        assertEquals(1, counter.get());
    }

    @Test
    void schedulerInterruptLoopTest() {
        AtomicInteger counter = new AtomicInteger();
        Task task = Task.loop(() -> {})
                .mutate()
                .onInterrupt(counter::incrementAndGet);
        Scheduler.schedule(task);
        task.finish();
        assertEquals(1, counter.get());
    }

    @Test
    void schedulerInterruptLambdaTest() {
        AtomicInteger counter = new AtomicInteger();
        BunyipsSubsystem subsystem = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task task = Task.loop(() -> {})
                .onInterrupt(counter::incrementAndGet)
                .on(subsystem);
        Task interruptor = new Lambda()
                .mutate()
                .onInterrupt(counter::incrementAndGet)
                .on(subsystem)
                .asPriority();
        Scheduler.schedule(task);
        Scheduler.schedule(interruptor);
        assertEquals(1, counter.get());
    }

    @Test
    void schedulerInterruptCauseLambdaInRunLoopTest() {
        AtomicInteger counter = new AtomicInteger();

        BunyipsSubsystem subsystem = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Task task = Task.loop(() -> {})
                .onInterrupt(counter::incrementAndGet)
                .on(subsystem);
        Task interruptor = new Lambda()
                .mutate()
                .onInterrupt(counter::incrementAndGet)
                .on(subsystem)
                .asPriority();

        Task interruptorScheduler = new Lambda(() -> Scheduler.schedule(interruptor));

        Scheduler.schedule(task);
        Scheduler.schedule(interruptorScheduler);

        Scheduler.update();

        assertEquals(1, counter.get());
    }

    @Test
    void registerSubsystemTest() {
        AtomicInteger counter = new AtomicInteger(0);
        BunyipsSubsystem system = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
                counter.incrementAndGet();
            }
        };
        Scheduler.update();
        assertEquals(1, counter.get());
    }

    @Test
    void unregisterSubsystemTest() {
        AtomicInteger counter = new AtomicInteger(0);
        BunyipsSubsystem unused = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        BunyipsSubsystem system = new BunyipsSubsystem() {
            @Override
            protected void periodic() {
                counter.incrementAndGet();
            }
        };
        Scheduler.use(unused);
        Scheduler.update();
        assertEquals(0, counter.get());
    }

    @Test
    void schedulerCancelAllTest() {
        AtomicInteger counter = new AtomicInteger();

        Task task = new IdleTask()
                .mutate().onInterrupt(counter::incrementAndGet);
        Task task2 = new IdleTask()
                .mutate().onInterrupt(counter::incrementAndGet);

        Scheduler.schedule(task);
        Scheduler.schedule(task2);
        Scheduler.finishAll();

        assertEquals(2, counter.get());
    }

    @Test
    void scheduleScheduledNoOp() {
        AtomicInteger counter = new AtomicInteger();

        Task task = new Lambda(counter::incrementAndGet);

        Scheduler.schedule(task);
        Scheduler.schedule(task);

        assertEquals(1, counter.get());
    }
}