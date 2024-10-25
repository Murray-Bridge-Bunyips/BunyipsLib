package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;

class UnitTest { // :)
    @Test
    void testOf() {
        ExampleUnit u = new ExampleUnit(1);
        Measure<ExampleUnit> fiveOfSomething = u.of(5);
        assertEquals(5, fiveOfSomething.magnitude(), 0);
        assertEquals(u, fiveOfSomething.unit());
    }
}