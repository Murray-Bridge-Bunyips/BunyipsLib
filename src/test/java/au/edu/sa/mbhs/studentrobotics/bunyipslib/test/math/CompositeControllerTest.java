package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.CompositeController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kA;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kG;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kS;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kV;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;

class CompositeControllerTest {
    private double vel;

    @Test
    void testComposedOutput() {
        PIDController a = new PIDController(0.1, 0.2, 0.3);
        PIDController b = new PIDController(10, 0.5, 0.6);

        CompositeController c = a.compose(b, Double::sum);
        assertEquals(10.1, c.calculate(1, 2));
    }

    @Test
    void testCoefficients() {
        PIDController a = new PController(0);
        PIDController b = new PController(0);
        CompositeController c = a.compose(b, (d, e) -> 0);

        a.setCoefficients(3.0, 5.0, 7.0);
        b.setCoefficients(11.0, 13.0, 17.0);
        assertArrayEquals(new double[]{3.0, 5.0, 7.0, 0.0, 11.0, 13.0, 17.0, 0.0}, c.getCoefficients());

        c.setCoefficients(1.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0, 0.0);
        assertArrayEquals(new double[]{1.0, 2.0, 3.0, 0.0}, a.getCoefficients());
        assertArrayEquals(new double[]{4.0, 5.0, 6.0, 0.0}, b.getCoefficients());
    }

    @Test
    void testCoefficientsWithFeedforward() {
        SystemController a = new kV(0.5);
        SystemController b = new PDController(0.6, 1);
        SystemController c = new kG(0.2).compose(new kS(0.9));
        SystemController all = a.compose(b.compose(c));

        assertArrayEquals(new double[]{0.5, 0.6, 0.0, 1.0, 0.0, 0.2, 0.9}, all.getCoefficients());
        a.setCoefficients(1.0);
        b.setCoefficients(1.0, 2.0, 3.0, 4.0);
        assertArrayEquals(new double[]{1.0, 1.0, 2.0, 3.0, 4.0, 0.2, 0.9}, all.getCoefficients());
        all.setCoefficients(1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0);
        assertArrayEquals(new double[]{2.0, 2.0}, c.getCoefficients());
        assertEquals(3, a.compose(c).getCoefficients().length);
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
        SystemController ff = new kS(0.5).compose(new kG(1)).compose(new kV(1.5)).compose(new kA(2));
        SystemController ff2 = new kS(0.5).compose(new kG(1)).compose(new kV(1.5)).compose(new kA(2));
        CompositeController c = a.compose(ff2, Double::sum);
        for (int i = 0; i < 100; i++) {
            vel = i;
            assertEquals(
                    a.calculate(i, i + 10) + ff.calculate(0, i + 10),
                    c.calculate(i, i + 10),
                    10
            );
            a.reset(); // multiple calculations of the same controller with derivative so we need to reset
        }
    }

    @Test
    void testUnlinkedPIDFF() {
        PIDController a = new PIDController(0.1, 1, 0); // derivative terms are based on delta time which is *not* consistent
        SystemController ff = new kS(0.5).compose(new kG(1)).compose(new kV(1.5)).compose(new kA(2));

        PIDController aUnlinked = new PIDController(0.1, 1, 0);
        SystemController ffUnlinked = new kS(0.5).compose(new kG(1)).compose(new kV(1.5)).compose(new kA(2));

        CompositeController c = aUnlinked.compose(ffUnlinked, Double::sum);
        for (int i = 0; i < 100; i++) {
            assertEquals(
                    a.calculate(i, i + 10) + ff.calculate(0, i + 10),
                    c.calculate(i, i + 10),
                    10
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
        CompositeController c = p.compose(new kG(0.3), Double::sum);
        assertEquals(0.3, c.calculate(0, 0));
        assertEquals(10.3, c.calculate(0, 10));
    }
}
