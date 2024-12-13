package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;

class PIDToleranceTest {
    private static final double kSetpoint = 50.0;
    private static final double kTolerance = 10.0;
    private static final double kRange = 200;

    @Test
    void testInitialTolerance() {
        var controller = new PIDController(0.05, 0.0, 0.0);
        controller.enableContinuousInput(-kRange / 2, kRange / 2);
        assertFalse(controller.atSetpoint());
    }

    @Test
    void testAbsoluteTolerance() {
        var controller = new PIDController(0.05, 0.0, 0.0);
        controller.enableContinuousInput(-kRange / 2, kRange / 2);

        assertFalse(controller.atSetpoint());

        controller.setTolerance(kTolerance);
        controller.setSetpoint(kSetpoint);

        assertFalse(controller.atSetpoint(),
                "Error was in tolerance when it should not have been. Error was "
                        + controller.getError());

        controller.calculate(0.0);

        assertFalse(controller.atSetpoint(),
                "Error was in tolerance when it should not have been. Error was "
                        + controller.getError());

        controller.calculate(kSetpoint + kTolerance / 2);

        assertTrue(controller.atSetpoint(),
                "Error was not in tolerance when it should have been. Error was "
                        + controller.getError());

        controller.calculate(kSetpoint + 10 * kTolerance);

        assertFalse(controller.atSetpoint(),
                "Error was in tolerance when it should not have been. Error was "
                        + controller.getError());
    }
}