package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Mult;

class MultTest {
    @Test
    void testAutomaticNames() {
        ExampleUnit unitA = new ExampleUnit(1, "Ay", "a");
        ExampleUnit unitB = new ExampleUnit(1, "Bee", "b");
        Mult<ExampleUnit, ExampleUnit> mult = Mult.combine(unitA, unitB);
        assertEquals("Ay-Bee", mult.name());
        assertEquals("a*b", mult.symbol());
    }

    @Test
    void testCombine() {
        ExampleUnit unitA = new ExampleUnit(100);
        ExampleUnit unitB = new ExampleUnit(0.912);
        Mult<ExampleUnit, ExampleUnit> mult = Mult.combine(unitA, unitB);
        assertEquals(91.2, mult.toBaseUnits(1));
    }

    @Test
    void testCaches() {
        ExampleUnit unitA = new ExampleUnit(1);
        ExampleUnit unitB = new ExampleUnit(2);
        Mult<ExampleUnit, ExampleUnit> mult = Mult.combine(unitA, unitB);
        assertSame(mult, Mult.combine(unitA, unitB));
    }
}