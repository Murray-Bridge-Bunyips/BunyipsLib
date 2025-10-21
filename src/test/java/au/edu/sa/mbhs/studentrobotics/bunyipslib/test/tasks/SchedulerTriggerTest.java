package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.concurrent.atomic.AtomicBoolean;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

class SchedulerTriggerTest {
    @BeforeEach
    void setUp() throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        Method cleanup = Scheduler.class.getDeclaredMethod("cleanup");
        cleanup.setAccessible(true);
        cleanup.invoke(null);
        Dbg.INSTANCE.setINHIBIT(true);
        Dashboard.INHIBIT_PACKETS = true;
        DualTelemetry.Companion.setINHIBIT_SMART_CALLS(true);
        // Required to register
        new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
    }

    @Test
    void onTrueTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.task().isFinished(finished::get);
        Gamepad gamepad = new Gamepad();

        gamepad.a = false;
        // Can't access implicit methods
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.A)
                .onTrue(task);
        Scheduler.update();
        assertFalse(task.isRunning());
        gamepad.a = true;
        Scheduler.update();
        assertTrue(task.isRunning());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    @Test
    void onFalseTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.task().isFinished(finished::get);
        Gamepad gamepad = new Gamepad();

        gamepad.b = true;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.B)
                .onFalse(task);
        Scheduler.update();
        assertFalse(task.isRunning());
        gamepad.b = false;
        Scheduler.update();
        assertTrue(task.isRunning());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    @Test
    void onChangeTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.task().isFinished(finished::get).named("taska");
        Gamepad gamepad = new Gamepad();

        gamepad.x = true;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.X)
                .onChange(task);
        Scheduler.update();
        assertFalse(task.isRunning());
        gamepad.x = false;
        Scheduler.update();
        assertTrue(task.isRunning());
        finished.set(true);
        Scheduler.update(); // task finish condition stops running the task
        assertFalse(task.isRunning());
        Scheduler.update(); // task onFinish fires and is removed
        assertFalse(Scheduler.activeTasks.contains(task));
        finished.set(false);
        gamepad.x = true;
        Scheduler.update(); // rescheduled
        assertTrue(task.isRunning());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    // TODO
}
