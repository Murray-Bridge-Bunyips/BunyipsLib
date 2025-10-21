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
        // Required to register
        new BunyipsSubsystem() {
            @Override
            protected void periodic() {
            }
        };
        Scheduler.update();
        assertFalse(task.isRunning());
        gamepad.a = true;
        Scheduler.update();
        assertTrue(task.isRunning());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isRunning());
    }

    // TODO
}
