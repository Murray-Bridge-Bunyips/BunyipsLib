package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Centimeters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.FeetPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millisecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millivolts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Volts;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.MutableMeasure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Voltage;

class MutableMeasureTest {
    @Test
    void testWrapper() {
        ExampleUnit unit = new ExampleUnit(1);
        Measure<ExampleUnit> measure = unit.of(1234);
        MutableMeasure<ExampleUnit> mutable = MutableMeasure.mutable(measure);
        assertEquals(measure.magnitude(), mutable.magnitude(), 0);
        assertEquals(measure.unit(), mutable.unit());
    }

    @Test
    void testSetMagnitude() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(0, FeetPerSecond);
        double newMagnitude = 48.123;
        measure.mut_setMagnitude(newMagnitude);
        assertEquals(newMagnitude, measure.magnitude(), 0); // should be an exact match
        double feetToMeters = 0.3048;
        assertEquals(newMagnitude * feetToMeters, measure.baseUnitMagnitude(), 1.0e-12);
    }

    @Test
    void testMut_times() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(18, FeetPerSecond);
        double scalar = Math.PI;
        MutableMeasure<Velocity<Distance>> result = measure.mut_times(scalar);
        assertSame(measure, result, "mut_times should return the same object");
        assertEquals(18 * Math.PI, measure.magnitude(), 1.0e-12);
    }

    @Test
    void testMut_divide() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(18, FeetPerSecond);
        double scalar = Math.PI;
        MutableMeasure<Velocity<Distance>> result = measure.mut_divide(scalar);
        assertSame(measure, result, "mut_times should return the same object");
        assertEquals(18 / Math.PI, measure.magnitude(), 1.0e-12);
    }

    @Test
    void testReplaceMeasure() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(17.6, FeetPerSecond);
        Measure<Velocity<Distance>> replacer = Meters.per(Millisecond).of(94.872);
        MutableMeasure<Velocity<Distance>> result = measure.mut_replace(replacer);
        assertSame(measure, result, "Replacing should return the mutable measure");
        assertSame(replacer.unit(), measure.unit());
        assertEquals(replacer.magnitude(), measure.magnitude(), 0);
    }

    @Test
    void testReplaceRaw() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(-542, FeetPerSecond);
        MutableMeasure<Velocity<Distance>> result = measure.mut_replace(62, Meters.per(Millisecond));
        assertSame(measure, result, "Replacing should return the mutable measure");
        assertSame(Meters.per(Millisecond), measure.unit());
        assertEquals(62, measure.magnitude(), 0);
    }

    @Test
    void testAccMeasure() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(8.5431, FeetPerSecond);
        Measure<Velocity<Distance>> acc = Meters.per(Millisecond).of(-23.62);
        MutableMeasure<Velocity<Distance>> result = measure.mut_acc(acc);
        assertSame(measure, result, "Acc should return the mutable measure");
        assertSame(FeetPerSecond, measure.unit(), "Unit shouldn't change");
        assertEquals(8.5431 - (23.62 * (1 / 0.3048) * 1000), measure.magnitude(), 1.0e-10);
    }

    @Test
    void testAccRaw() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(99.999999, FeetPerSecond);
        MutableMeasure<Velocity<Distance>> result = measure.mut_acc(22.981);
        assertSame(measure, result);
        assertSame(FeetPerSecond, measure.unit());
        assertEquals(122.980999, measure.magnitude(), 0);
    }

    @Test
    void testMutPlusMeasure() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(400, InchesPerSecond);
        Measure<Velocity<Distance>> other = Centimeters.per(Second).of(41.312);
        MutableMeasure<Velocity<Distance>> result = measure.mut_plus(other);
        assertSame(measure, result);
        assertSame(InchesPerSecond, result.unit());
        assertEquals(416.2645669291339, measure.magnitude(), 1.0e-12);
    }

    @Test
    void testMutPlusRaw() {
        MutableMeasure<Voltage> measure = MutableMeasure.ofRelativeUnits(31.51, Volts);
        Measure<Voltage> result = measure.mut_plus(66.641, Millivolts);
        assertSame(measure, result);
        assertSame(Volts, result.unit());
        assertEquals(31.576641, result.magnitude(), 1.0e-10);
    }

    @Test
    void testMutMinusMeasure() {
        MutableMeasure<Velocity<Distance>> measure = MutableMeasure.ofRelativeUnits(400, InchesPerSecond);
        Measure<Velocity<Distance>> other = Centimeters.per(Second).of(41.312);
        MutableMeasure<Velocity<Distance>> result = measure.mut_minus(other);
        assertSame(measure, result);
        assertSame(InchesPerSecond, result.unit());
        assertEquals(383.7354330708662, measure.magnitude(), 1.0e-12);
    }

    @Test
    void testMutMinusRaw() {
        MutableMeasure<Voltage> measure = MutableMeasure.ofRelativeUnits(31.51, Volts);
        MutableMeasure<Voltage> result = measure.mut_minus(66.641, Millivolts);
        assertSame(measure, result);
        assertSame(Volts, result.unit());
        assertEquals(31.443359, result.magnitude(), 1.0e-10);
    }
}