package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Current;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Dimensionless;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Mass;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Per;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Unit;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Voltage;

class MeasureTest {
    @Test
    void testBasics() {
        Unit<Distance> unit = Units.Feet;
        double magnitude = 10;
        Measure<Distance> m = unit.of(magnitude);
        assertEquals(unit, m.unit(), "Wrong units");
        assertEquals(magnitude, m.magnitude(), 0, "Wrong magnitude");
    }

    @Test
    void testMultiply() {
        Measure<Distance> m = Units.Feet.of(1);
        Measure<Distance> m2 = m.times(10);
        assertEquals(10, m2.magnitude(), 1.0e-12);
        assertNotSame(m2, m); // make sure state wasn't changed
    }

    @Test
    void testDivide() {
        Measure<Distance> m = Units.Meters.of(1);
        Measure<Distance> m2 = m.div(10);
        assertEquals(0.1, m2.magnitude(), 0);
        assertNotSame(m2, m);
    }

    @Test
    void testAdd() {
        Measure<Distance> m1 = Units.Feet.of(1);
        Measure<Distance> m2 = Units.Inches.of(2);
        assertTrue(m1.plus(m2).isEquivalent(Units.Feet.of(1 + 2 / 12.0d)));
        assertTrue(m2.plus(m1).isEquivalent(Units.Inches.of(14)));
    }

    @Test
    void testSubtract() {
        Measure<Distance> m1 = Units.Feet.of(1);
        Measure<Distance> m2 = Units.Inches.of(2);
        assertTrue(m1.minus(m2).isEquivalent(Units.Feet.of(1 - 2 / 12.0d)));
        assertTrue(m2.minus(m1).isEquivalent(Units.Inches.of(-10)));
    }

    @Test
    void testNegate() {
        Measure<Distance> m = Units.Feet.of(123);
        Measure<Distance> n = m.negate();
        assertEquals(-m.magnitude(), n.magnitude(), 1.0e-12);
        assertEquals(m.unit(), n.unit());
    }

    @Test
    void testEquivalency() {
        Measure<Distance> inches = Units.Inches.of(12);
        Measure<Distance> feet = Units.Feet.of(1);
        assertTrue(inches.isEquivalent(feet));
        assertTrue(feet.isEquivalent(inches));
    }

    @Test
    void testAs() {
        Measure<Distance> m = Units.Inches.of(12);
        assertEquals(1, m.in(Units.Feet), Measure.EQUIVALENCE_THRESHOLD);
    }

    @Test
    void testPerMeasureTime() {
        Measure<Mass> measure = Units.Kilograms.of(144);
        Measure<Time> dt = Units.Milliseconds.of(53);

        // 144 Kg / (53 ms) = (1000 / 53) * 144 Kg/s = (144,000 / 53) Kg/s

        Measure<Velocity<Mass>> result = measure.per(dt);
        assertEquals(144_000.0 / 53, result.baseUnitMagnitude(), 1.0e-5);
        assertEquals(Units.Kilograms.per(Units.Milliseconds), result.unit());
    }

    @Test
    void testPerUnitTime() {
        Measure<Mass> measure = Units.Kilograms.of(144);
        Measure<Velocity<Mass>> result = measure.per(Units.Millisecond);

        assertSame(Velocity.class, result.unit().getClass());
        assertEquals(144_000.0, result.baseUnitMagnitude(), 1.0e-5);
        assertEquals(Units.Kilograms.per(Units.Milliseconds), result.unit());
    }

    @Test
    void testTimesMeasure() {
        Measure<Voltage> m1 = Units.Volts.of(1.567);
        Measure<Mass> m2 = Units.Kilograms.of(8.4e-5);

        assertEquals(Units.Volts.mult(Units.Kilograms).of(1.567 * 8.4e-5), m1.times(m2));
    }

    @Test
    void testTimesUnitless() {
        ExampleUnit unit = new ExampleUnit(6);
        Measure<ExampleUnit> measure = unit.of(2.5);
        Measure<Dimensionless> multiplier = Units.Percent.of(125); // 125% or 1.25x
        Measure<?> result = measure.times(multiplier);
        assertSame(unit, result.unit());

        assertEquals(2.5 * 1.25, result.magnitude());
        assertEquals(2.5 * 1.25 * 6, result.baseUnitMagnitude());
    }

    @Test
    void testTimesPerWithDimensionalAnalysis() {
        Measure<Distance> measureA = Units.Feet.of(62); // 62 feet
        Measure<Per<Angle, Distance>> measureB = Units.Radians.of(6).per(Units.Inches); // 6 radians per inch
        Measure<?> aTimesB = measureA.times(measureB); // (62 feet) * (6 rad/inch) = 4464 rad
        assertEquals(Units.Radians, aTimesB.unit());
        assertEquals(4464, aTimesB.magnitude(), 1.0e-12);

        Measure<?> bTimesA = measureB.times(measureA); // should be identical to the above
        assertTrue(bTimesA.isEquivalent(aTimesB));
        assertTrue(aTimesB.isEquivalent(bTimesA));
    }

    @Test
    void testPerTimesPerWithDimensionalAnalysis() {
        Measure<Per<Distance, Voltage>> measureA = Units.Inches.of(16).per(Units.Volts);
        Measure<Per<Voltage, Distance>> measureB = Units.Millivolts.of(14).per(Units.Meters);
        Measure<?> aTimesB = measureA.times(measureB);
        assertEquals(Units.Value, aTimesB.unit());
        assertEquals((16 * 25.4 / 1000) * (14 / 1000.0), aTimesB.magnitude());
        assertEquals((16 * 25.4 / 1000) * (14 / 1000.0), aTimesB.baseUnitMagnitude());

        Measure<?> bTimesA = measureB.times(measureA); // should be identical to the above
        assertTrue(bTimesA.isEquivalent(aTimesB));
        assertTrue(aTimesB.isEquivalent(bTimesA));
    }

    @Test
    void testPerTimesMeasure() {
        Measure<Velocity<Distance>> m1 = Units.Feet.per(Units.Milliseconds).of(19);
        Measure<Time> m2 = Units.Seconds.of(44);

        // 19 ft/ms = 19,000 ft/s
        // 19,000 ft/s * 44s = 836,000 ft
        assertTrue(Units.Feet.of(836_000).isNear(m1.times(m2), 1.0e-12));

        // 42 ex per foot * 17mm = 42 ex * 17mm / (304.8mm/ft) = 42 * 17 / 304.8 = 2.34252
        ExampleUnit exampleUnit = new ExampleUnit(1);
        Measure<Per<ExampleUnit, Distance>> m3 = exampleUnit.per(Units.Feet).of(42);
        Measure<Distance> m4 = Units.Millimeters.of(17);
        assertEquals(exampleUnit.of(42 * 17 / (12 * 25.4)), m3.times(m4));
    }

    @Test
    void testDivideMeasure() {
        // Dimensionless divide
        Measure<Distance> m1 = Units.Meters.of(6);
        Measure<Dimensionless> m2 = Units.Value.of(3);
        Measure<?> result = m1.div(m2);
        assertEquals(m1.div(m2).magnitude(), 2);
        assertEquals(result.unit(), Units.Meters);
        // Velocity divide
        Measure<Distance> m3 = Units.Meters.of(8);
        Measure<Velocity<Distance>> m4 = Units.Meters.per(Units.Second).of(4);
        result = m3.div(m4);
        assertEquals(result.magnitude(), 2);
        assertEquals(result.unit(), Units.Second);
        // Per divide
        Measure<Voltage> m5 = Units.Volts.of(6);
        Measure<Per<Voltage, Distance>> m6 = Units.Volts.per(Units.Meter).of(2);
        result = m5.div(m6);
        assertEquals(result.magnitude(), 3);
        assertEquals(result.unit(), Units.Meter);
        // Fallthrough divide
        Measure<Time> m7 = Units.Seconds.of(10);
        Measure<Current> m8 = Units.Amps.of(2);
        result = m7.div(m8);
        assertEquals(result.magnitude(), 5);
        assertEquals(result.unit(), Units.Seconds.per(Units.Amps));
        // Same base unit divide
        Measure<Distance> m9 = Units.Meters.of(8);
        Measure<Distance> m10 = Units.Meters.of(4);
        result = m9.div(m10);
        assertEquals(result.magnitude(), 2);
        assertEquals(result.unit(), Units.Value);
    }

    @Test
    void testToShortString() {
        Measure<Voltage> measure = Units.Volts.of(343);
        assertEquals("343.0 V", measure.toShortString());
    }

    @Test
    void testToScientificString() {
        Measure<Voltage> measure = Units.Volts.of(343);
        assertEquals("3.430e+02 V", measure.toScientificString());
    }

    @Test
    void testToLongString() {
        Measure<Voltage> measure = Units.Volts.of(343);
        assertEquals("343.0 Volt", measure.toLongString());
        assertEquals("343.0001 Volt", Units.Volts.of(343.0001).toLongString());
        assertEquals("1.2345678912345678E8 Volt", Units.Volts.of(123456789.12345678).toLongString());
    }

    @Test
    void testOfBaseUnits() {
        ExampleUnit unit = new ExampleUnit(16);
        Measure<ExampleUnit> measure = unit.ofBaseUnits(1);
        assertEquals(unit, measure.unit());
        assertEquals(1, measure.baseUnitMagnitude());
        assertEquals(1 / 16.0, measure.magnitude());
    }

    @Test
    @SuppressWarnings("EqualsWithItself")
    void testCompare() {
        ExampleUnit unit = new ExampleUnit(7);
        Measure<ExampleUnit> base = unit.of(1);
        Measure<ExampleUnit> less = unit.of(0.5);
        Measure<ExampleUnit> more = unit.of(2);

        assertEquals(0, base.compareTo(base));
        assertEquals(-1, base.compareTo(more));
        assertEquals(1, base.compareTo(less));

        // lt, lte, gt, gte helper functions

        assertTrue(base.lt(more));
        assertTrue(base.lte(more));
        assertFalse(base.gt(more));
        assertFalse(base.gte(more));

        assertTrue(base.gt(less));
        assertTrue(base.gte(less));
        assertFalse(base.lt(less));
        assertFalse(base.lte(less));

        assertTrue(base.lte(base));
        assertTrue(base.gte(base));
        assertFalse(base.lt(base));
        assertFalse(base.gt(base));
    }

    @Test
    void testTimesScalar() {
        ExampleUnit unit = new ExampleUnit(42);
        Measure<ExampleUnit> measure = unit.of(4.2);
        double scalar = 18;
        Measure<ExampleUnit> result = measure.times(scalar);
        assertNotSame(measure, result);
        assertSame(unit, result.unit());
        assertEquals(4.2 * 18, result.magnitude());
        assertEquals(4.2 * 42 * 18, result.baseUnitMagnitude());
    }

    @Test
    void testPerUnit() {
        ExampleUnit unitA = new ExampleUnit(10);
        ExampleUnit unitB = new ExampleUnit(12);
        Measure<ExampleUnit> measure = unitA.of(1.2);
        Measure<Per<ExampleUnit, ExampleUnit>> result = measure.per(unitB);
        assertEquals(Per.combine(unitA, unitB), result.unit()); // A/B has base equivalent of 10/12
        assertEquals(1, result.baseUnitMagnitude()); // 10/12 * 12/10 = 1
        assertEquals(measure.magnitude(), result.magnitude());
    }

    @Test
    void testAddMeasureSameUnit() {
        ExampleUnit unit = new ExampleUnit(8.2);
        Measure<ExampleUnit> measureA = unit.of(3.1);
        Measure<ExampleUnit> measureB = unit.of(91.6);
        Measure<ExampleUnit> result = measureA.plus(measureB);
        assertEquals(unit, result.unit());
        assertEquals(94.7, result.magnitude(), 1.0e-12);
    }

    @Test
    void testAddMeasuresDifferentUnits() {
        ExampleUnit unitA = new ExampleUnit(8.2);
        ExampleUnit unitB = new ExampleUnit(7.3);
        Measure<ExampleUnit> measureA = unitA.of(5);
        Measure<ExampleUnit> measureB = unitB.of(16);
        Measure<ExampleUnit> aPlusB = measureA.plus(measureB);

        assertEquals(unitA, aPlusB.unit());
        assertEquals(8.2 * 5 + 7.3 * 16, aPlusB.baseUnitMagnitude(), 1.0e-12);
        assertEquals(5 + (16 * 7.3 / 8.2), aPlusB.magnitude(), 1.0e-12);

        Measure<ExampleUnit> bPlusA = measureB.plus(measureA);
        assertEquals(unitB, bPlusA.unit());
        assertEquals(8.2 * 5 + 7.3 * 16, bPlusA.baseUnitMagnitude(), 1.0e-12);
        assertEquals(16 + (5 * 8.2 / 7.3), bPlusA.magnitude(), 1.0e-12);
    }

    @Test
    void testMinNoArgs() {
        Measure<?> min = Measure.min();
        assertNull(min);
    }

    @Test
    void testMin() {
        ExampleUnit unit = new ExampleUnit(56.1);
        Measure<ExampleUnit> one = unit.of(1);
        Measure<ExampleUnit> two = unit.of(2);
        Measure<ExampleUnit> zero = unit.of(0);
        Measure<ExampleUnit> veryNegative = unit.of(-12839712);

        Measure<ExampleUnit> min = Measure.min(one, two, zero, veryNegative);
        assertSame(veryNegative, min);
    }

    @Test
    void testMaxNoArgs() {
        Measure<?> min = Measure.max();
        assertNull(min);
    }

    @Test
    void testMax() {
        ExampleUnit unit = new ExampleUnit(6.551);
        Measure<ExampleUnit> one = unit.of(1);
        Measure<ExampleUnit> two = unit.of(2);
        Measure<ExampleUnit> zero = unit.of(0);
        Measure<ExampleUnit> veryLarge = unit.of(8217234);

        Measure<ExampleUnit> max = Measure.max(one, two, zero, veryLarge);
        assertSame(veryLarge, max);
    }

    @Test
    void testIsNearVarianceThreshold() {
        ExampleUnit unit = new ExampleUnit(92);
        Measure<ExampleUnit> measureA = unit.of(1.21);
        Measure<ExampleUnit> measureB = unit.ofBaseUnits(64);
        // A = 1.21 * 92 base units, or 111.32
        // B = 64 base units
        // ratio = 111.32 / 64 = 1.739375 = 173.9375%

        assertTrue(measureA.isNear(measureA, 0));
        assertTrue(measureB.isNear(measureB, 0));

        assertFalse(measureA.isNear(measureB, 0));
        assertFalse(measureA.isNear(measureB, 0.50));
        assertFalse(measureA.isNear(measureB, 0.739370));
        assertTrue(measureA.isNear(measureB, 0.739375));
        assertTrue(measureA.isNear(measureB, 100)); // some stupidly large range +/- 10000%

        Measure<ExampleUnit> measureC = unit.of(-1.21);
        Measure<ExampleUnit> measureD = unit.ofBaseUnits(-64);

        assertTrue(measureC.isNear(measureC, 0));
        assertTrue(measureD.isNear(measureD, 0));

        assertFalse(measureC.isNear(measureD, 0));
        assertFalse(measureC.isNear(measureD, 0.50));
        assertFalse(measureC.isNear(measureD, 0.739370));
        assertTrue(measureC.isNear(measureD, 0.739375));
        assertTrue(measureC.isNear(measureD, 100)); // some stupidly large range +/- 10000%

        Measure<Distance> measureE = Units.Meters.of(1);
        Measure<Distance> measureF = Units.Feet.of(-3.28084);

        assertTrue(measureE.isNear(measureF, 2.01));
        assertFalse(measureE.isNear(measureF, 1.99));

        assertTrue(measureF.isNear(measureE, 2.01));
        assertFalse(measureF.isNear(measureE, 1.99));

        assertTrue(Units.Feet.zero().isNear(Units.Millimeters.zero(), 0.001));
        assertFalse(Units.Feet.of(2).isNear(Units.Millimeters.of(0), 0.001));
    }

    @Test
    void testIsNearMeasureTolerance() {
        Measure<Distance> measureCompared = Units.Meters.of(1);
        Measure<Distance> measureComparing = Units.Meters.of(1.2);

        // Positive value with positive tolerance
        assertTrue(measureCompared.isNear(measureComparing, Units.Millimeters.of(300)));
        assertFalse(measureCompared.isNear(measureComparing, Units.Centimeters.of(10)));

        measureCompared = measureCompared.negate();
        measureComparing = measureComparing.negate();

        // Negative value with positive tolerance
        assertTrue(measureCompared.isNear(measureComparing, Units.Millimeters.of(300)));
        assertFalse(measureCompared.isNear(measureComparing, Units.Centimeters.of(10)));

        measureCompared = measureCompared.negate();
        measureComparing = measureComparing.negate();

        // Positive value with negative tolerance
        assertTrue(measureCompared.isNear(measureComparing, Units.Millimeters.of(-300)));
        assertFalse(measureCompared.isNear(measureComparing, Units.Centimeters.of(-10)));

        measureCompared = measureCompared.negate();
        measureComparing = measureComparing.negate();

        // Negative value with negative tolerance.
        assertTrue(measureCompared.isNear(measureComparing, Units.Millimeters.of(-300)));
        assertFalse(measureCompared.isNear(measureComparing, Units.Centimeters.of(-10)));

        measureCompared = measureCompared.negate();
        measureComparing = measureComparing.negate();

        // Tolerance exact difference between measures.
        assertTrue(measureCompared.isNear(measureComparing, Units.Millimeters.of(200)));
        assertTrue(measureCompared.isNear(measureComparing, Units.Centimeters.of(-20)));
    }
}