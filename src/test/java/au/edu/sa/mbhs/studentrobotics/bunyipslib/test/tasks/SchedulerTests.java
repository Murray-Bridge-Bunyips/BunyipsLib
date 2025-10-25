package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import org.junit.jupiter.api.BeforeEach;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

abstract class SchedulerTests {
    @BeforeEach
    void setUp() throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
        Method cleanup = Scheduler.class.getDeclaredMethod("cleanup");
        cleanup.setAccessible(true);
        cleanup.invoke(null);
        Method reset = BunyipsSubsystem.class.getDeclaredMethod("reset");
        reset.setAccessible(true);
        reset.invoke(null);
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
}
