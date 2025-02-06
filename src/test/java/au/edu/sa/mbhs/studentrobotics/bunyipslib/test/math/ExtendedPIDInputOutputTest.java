package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;

class ExtendedPIDInputOutputTest {
    @Test
    void testContinuousInput() {
        var controller = new PIDController(0.0, 0.0, 0.0);

        controller.setP(1);
        controller.enableContinuousInput(-180, 180);
        assertEquals(controller.calculate(-179, 179), -2, 1.0e-5);

        controller.enableContinuousInput(0, 360);
        assertEquals(controller.calculate(1, 359), -2, 1.0e-5);
    }

    @Test
    void testProportionalGainOutput() {
        var controller = new PIDController(0.0, 0.0, 0.0);

        controller.setP(4);

        assertEquals(-0.1, controller.calculate(0.025, 0), 1.0e-5);
    }

    @Test
    void testIntegralGainOutput() {
        var controller = new PIDController(0.0, 0.0, 0.0);

        controller.setI(4);

        double out = 0;

        for (int i = 0; i < 5; i++) {
            out = controller.calculate(0.025, 0);
        }

        assertEquals(-0.5 * controller.getPeriod(), out, 1.0e-5);
    }

    @Test
    void testIntegralResetOnNewSetpoint() {
        var controller = new PIDController(0.0, 0.0, 0.0);
        controller.setClearIntegralOnNewSetpoint(true);
        controller.setI(4);
        for (int i = 0; i < 5; i++)
            controller.calculate(0.025, 0);
        assertEquals(0, controller.calculate(0.025, 0.01), 1.0e-5);
    }

    @Test
    void testClampedUpperOutput() {
        var controller = new PIDController(1.0, 0.0, 0.0);
        controller.upperClamp = 1;
        assertEquals(1, controller.calculate(0, 100));
    }

    @Test
    void testClampedLowerOutput() {
        var controller = new PIDController(1.0, 0.0, 0.0);
        controller.lowerClamp = -1;
        assertEquals(-1, controller.calculate(0, -100));
    }

    @Test
    void testIZoneNoOutput() {
        var controller = new PIDController(0.0, 0.0, 0.0);

        controller.setI(1);
        controller.setIntegrationZone(1);

        double out = controller.calculate(2, 0);

        assertEquals(0, out, 1.0e-5);
    }

    @Test
    void testIZoneOutput() {
        var controller = new PIDController(0.0, 0.0, 0.0);

        controller.setI(1);
        controller.setIntegrationZone(1);

        double out = controller.calculate(1, 0);

        assertEquals(-1 * controller.getPeriod(), out, 1.0e-5);
    }
}
