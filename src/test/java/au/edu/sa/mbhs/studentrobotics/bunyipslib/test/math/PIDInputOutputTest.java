package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;

class PIDInputOutputTest {
    private PIDController controller;

    @BeforeEach
    void setUp() {
        controller = new PIDController(0, 0, 0);
    }

    @Test
    void testProportionalGainOutput() {
        controller.setP(4);

        controller.setSetpoint(1000);
        controller.setTolerance(1);
        assertFalse(controller.atSetpoint());

        assertEquals(-0.1, controller.calculate(0.025, 0), 1.0e-5);
    }

    @Test
    void testIntegralGainOutput() {
        controller.setI(4);
        controller.setIntegrationBounds(-180, 180);

        double out = 0;

        for (int i = 0; i < 5; i++) {
            out = controller.calculate(0.025, 0);
        }

        assertEquals(-0.5 * controller.getPeriod(), out, 1.0e-5);
    }

    @Test
    void testDerivativeGainOutput() {
        controller.setD(4);
        controller.setDerivativeSmoothingGain(Double.MIN_VALUE);

        controller.setSetpoint(1000);
        controller.setTolerance(Double.MAX_VALUE, 1);
        assertFalse(controller.atSetpoint());

        assertEquals(0, controller.calculate(0, 0));

        assertEquals(controller.calculate(0.025, 0), -0.1 / controller.getPeriod(), 0.05);
    }

    @Test
    void testErrorTolerancePeriod() {
        controller.setP(0.5);
        assertEquals(0, controller.getPeriod());
        assertEquals(0, controller.getErrorDerivative());
        assertEquals(Double.POSITIVE_INFINITY, controller.getTolerance()[1]);
        controller.setSetpoint(100);
        assertEquals(100, controller.getError());
        assertEquals(0, controller.getPeriod());
        assertEquals(Double.POSITIVE_INFINITY, controller.getErrorDerivative());
        assertFalse(controller.atSetpoint());
        controller.setTolerance(Double.POSITIVE_INFINITY);
        assertFalse(controller.atSetpoint());
        controller.setTolerance(5);
        double value = 0;
        do {
            value = value + controller.calculate(value);
        } while (!controller.atSetpoint());
        assertTrue(controller.atSetpoint());
        controller.setSetpoint(0);
        assertFalse(controller.atSetpoint());
    }
}