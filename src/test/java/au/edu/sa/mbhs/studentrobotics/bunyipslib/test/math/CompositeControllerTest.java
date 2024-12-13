package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.CompositeController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.ElevatorFeedforward;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;

class CompositeControllerTest {
    private double vel;
    private double acc;

    @Test
    void testComposedOutput() {
        PIDController a = new PIDController(0.1, 0.2, 0.3);
        PIDController b = new PIDController(10, 0.5, 0.6);

        CompositeController c = a.compose(b, Double::sum);
        assertEquals(c.calculate(1, 2), 10.1);
    }

    @Test
    void testCoefficients() {
        PIDController a = new PController(0);
        PIDController b = new PController(0);
        CompositeController c = a.compose(b, (d, e) -> 0);

        a.setCoefficients(3.0, 5.0, 7.0);
        b.setCoefficients(11.0, 13.0, 17.0);
        assertArrayEquals(c.getCoefficients(), new double[]{3.0, 5.0, 7.0, 0.0, 11.0, 13.0, 17.0, 0.0});

        c.setCoefficients(1.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0, 0.0);
        assertArrayEquals(a.getCoefficients(), new double[]{1.0, 2.0, 3.0, 0.0});
        assertArrayEquals(b.getCoefficients(), new double[]{4.0, 5.0, 6.0, 0.0});
    }

    @Test
    void testSquid() {
        PIDController a = new PController(1);
        CompositeController squid = a.compose(SystemController.NULL, (ae, o) -> Math.sqrt(ae));
        assertEquals(2, squid.calculate(0, 4));
    }

    @Test
    void testLinkedPIDFF() {
        PIDController a = new PIDController(0.1, 0, 0.3);
        a.setDerivativeSmoothingGain(Double.MIN_VALUE); // cant use derivative lp gain due to linkage
        ElevatorFeedforward ff = new ElevatorFeedforward(0.5, 1, 1.5, 2, () -> vel, () -> acc);
        CompositeController c = a.compose(ff, Double::sum);
        for (int i = 0; i < 100; i++) {
            vel = i;
            acc = i;
            assertEquals(
                    a.calculate(i, i + 10) + ff.calculate(),
                    c.calculate(i, i + 10),
                    0.001
            );
            a.reset(); // multiple calculations of the same controller with derivative so we need to reset
        }
    }

    @Test
    void testUnlinkedPIDFF() {
        PIDController a = new PIDController(0.1, 1, 0); // derivative terms are based on delta time which is *not* consistent
        ElevatorFeedforward ff = new ElevatorFeedforward(0.5, 1, 1.5, 2, () -> vel, () -> acc);

        PIDController aUnlinked = new PIDController(0.1, 1, 0);
        ElevatorFeedforward ffUnlinked = new ElevatorFeedforward(0.5, 1, 1.5, 2, () -> vel, () -> acc);

        CompositeController c = aUnlinked.compose(ffUnlinked, Double::sum);
        for (int i = 0; i < 100; i++) {
            vel = i;
            acc = i;
            assertEquals(
                    a.calculate(i, i + 10) + ff.calculate(),
                    c.calculate(i, i + 10),
                    0.001
            );
        }
    }

    @Test
    void testPIDFInterface() {
        PIDController a = new PIDController(0.1, 0.2, 0.3);
        PIDController b = new PIDController(10, 0.5, 0.6);

        CompositeController c = a.compose(b, Double::sum);
        assertThrows(IllegalArgumentException.class, c::pidf);

        CompositeController d = a.compose(SystemController.NULL, Double::sum);
        assertTrue(d.pidf().isPresent());
        assertEquals(d.pidf().get(), a);
    }

    @Test
    void testkG() {
        PController p = new PController(1);
        CompositeController c = p.compose(new ElevatorFeedforward(0.0, 0.3, 0.0, 0.0, () -> 0, () -> 0), Double::sum);
        assertEquals(c.calculate(0, 0), 0.3);
        assertEquals(c.calculate(0, 10), 10.3);
    }
}
