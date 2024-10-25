package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.LookupTable;

class LookupTableTest {
    @Test
    void getClosest() {
        LookupTable<Double, Integer> lut = new LookupTable<>();
        lut.add(0.0, 0);
        lut.add(1.0, 1);
        lut.add(2.0, 2);

        for (int i = 0; i < 50; i++) {
            double d = Math.random() * 2.5; // generate number in range [0, 2.5)
            assertEquals(Math.round(d), lut.get(d).intValue());
        }
    }

    @Test
    void empty() {
        assertNull(new LookupTable<Integer, Integer>().get(0));
    }
}