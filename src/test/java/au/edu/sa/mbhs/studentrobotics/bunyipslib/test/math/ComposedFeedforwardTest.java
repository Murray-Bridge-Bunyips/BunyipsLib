package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kA;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kG;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kS;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.kV;

class ComposedFeedforwardTest {
    private static final double ks = 0.5;
    private static final double kg = 1;
    private static final double kv = 1.5;
    private static final double ka = 2;

    private final SystemController ff = new kS(ks).compose(new kG(kg)).compose(new kV(kv)).compose(new kA(ka));

    @Test
    void testCalculate() {
        assertEquals(1, ff.calculate(0, 0), 0.002);
        try {
            Thread.sleep(1000); // dt
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        assertEquals(8.5, ff.calculate(0, 2), 0.5);
    }
}
