package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Feet;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.FeetPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.MetersPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.MetersPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millisecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Mult;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Per;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

class VelocityTest {
    @Test
    void testBaseUnit() {
        assertTrue(MetersPerSecond.equivalent(MetersPerSecond));
        assertTrue(Meters.per(Second).equivalent(MetersPerSecond));
    }

    @Test
    void testToAcceleration() {
        Velocity<Velocity<Distance>> metersPerSecondPerMillisecond = MetersPerSecond.per(Millisecond);

        assertEquals(1000, metersPerSecondPerMillisecond.of(1).in(MetersPerSecondPerSecond), 0);
        assertEquals(0, metersPerSecondPerMillisecond.of(0).in(MetersPerSecondPerSecond), 0);
    }

    @Test
    void testCache() {
        assertSame(FeetPerSecond, Feet.per(Second), "Feet.per(Second) should return a cached object instance");

        // completely arbitrary units chosen because they won't have already been cached
        ExampleUnit someDistance = new ExampleUnit(5);
        ExampleUnit someTime = new ExampleUnit(600);
        Per<ExampleUnit, ExampleUnit> firstInvocation = someDistance.per(someTime);
        Per<ExampleUnit, ExampleUnit> secondInvocation = someDistance.per(someTime);
        assertSame(
                firstInvocation,
                secondInvocation,
                firstInvocation + " was not the same object as " + secondInvocation);
    }

    @Test
    void testMult() {
        ExampleUnit baseUnit = new ExampleUnit(92);
        Velocity<ExampleUnit> vel = baseUnit.per(Millisecond);
        Mult<Velocity<ExampleUnit>, Time> mult = vel.mult(Second);
        assertEquals(92_000, mult.toBaseUnits(1), 1.0e-5);
    }
}