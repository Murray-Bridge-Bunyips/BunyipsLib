package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.SelectTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

class SelectTaskTest extends SchedulerTests {
    @Test
    void selectTaskTest() {
        AtomicBoolean task1_init = new AtomicBoolean(), task1_periodic = new AtomicBoolean(), task1_onFinish = new AtomicBoolean();
        Task task1 = Task.task()
                .init(() -> task1_init.set(true))
                .periodic(() -> task1_periodic.set(true))
                .onFinish(() -> task1_onFinish.set(true))
                .isFinished(() -> true);
        AtomicBoolean task2_init = new AtomicBoolean(), task2_periodic = new AtomicBoolean(), task2_onFinish = new AtomicBoolean();
        Task task2 = Task.task()
                .init(() -> task2_init.set(true))
                .periodic(() -> task2_periodic.set(true))
                .onFinish(() -> task2_onFinish.set(true));
        AtomicBoolean task3_init = new AtomicBoolean(), task3_periodic = new AtomicBoolean(), task3_onFinish = new AtomicBoolean();
        Task task3 = Task.task()
                .init(() -> task3_init.set(true))
                .periodic(() -> task3_periodic.set(true))
                .onFinish(() -> task3_onFinish.set(true));

        SelectTask<String> selectTask = new SelectTask<>(() -> "one")
                .when("one", task1)
                .when("two", task2)
                .when("three", task3);

        Scheduler.schedule(selectTask);
        Scheduler.update();

        assertTrue(task1_init.get());
        assertTrue(task1_periodic.get());
        assertTrue(task1_onFinish.get());

        assertFalse(task2_init.get());
        assertFalse(task2_periodic.get());
        assertFalse(task2_onFinish.get());

        assertFalse(task3_init.get());
        assertFalse(task3_periodic.get());
        assertFalse(task3_onFinish.get());
    }

    @Test
    void selectTaskInvalidKeyTest() {
        Task task1 = Task.task().isFinished(() -> true);
        Task task2 = Task.task();
        Task task3 = Task.task();

        SelectTask<String> selectTask = new SelectTask<>(() -> "four")
                .when("one", task1)
                .when("two", task2)
                .when("three", task3);

        assertDoesNotThrow(() -> {
            Scheduler.schedule(selectTask);
            Scheduler.update();
        });
    }

    @Test
    void selectTaskRequirementTest() {
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

        AtomicBoolean f1 = new AtomicBoolean();
        Task task1 = Task.task()
                .onFinish(() -> f1.set(true))
                .on(system1);
        AtomicBoolean f2 = new AtomicBoolean();
        Task task2 = Task.task()
                .onFinish(() -> f2.set(true))
                .on(system2);
        AtomicBoolean f3 = new AtomicBoolean();
        Task task3 = Task.task()
                .onFinish(() -> f3.set(true))
                .on(system2);

        AtomicReference<String> sup = new AtomicReference<>("one");

        SelectTask<String> selectTask = new SelectTask<>(sup::get)
                .when("one", task1)
                .when("two", task2)
                .when("three", task3);

        Scheduler.schedule(selectTask);
        Scheduler.schedule(new Lambda().on(system2));

        // BunyipsLib behaviour, disabled attachments are not stopped from execution on dependency conflicts
        assertTrue(selectTask.isActive());

        sup.set("unknown");
        Scheduler.update();
        assertFalse(f1.get());
        assertFalse(f2.get());
        assertFalse(f3.get());

        sup.set("two");
        Scheduler.update();
        assertTrue(f1.get());
        assertFalse(f2.get());
        assertFalse(f3.get());
    }
}