package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Amps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Celsius;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Centimeters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Fahrenheit;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Feet;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.FeetPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Grams;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Gs;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Horsepower;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Kelvin;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Kilo;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Kilograms;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.MetersPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.MetersPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Microseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milli;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliamps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millimeters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millivolts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliwatts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Minutes;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Ounces;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Percent;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Pounds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Revolutions;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Value;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Volts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Watts;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Unit;

class UnitsTest {
    // Be accurate to 0.01%
    private static final double thresh = 1.0e-5;

    void testBaseUnit(Unit<?> baseUnit) {
        assertEquals(0, baseUnit.of(0).baseUnitMagnitude(), 0);
        assertEquals(1, baseUnit.of(1).baseUnitMagnitude(), 0);
        assertEquals(-1, baseUnit.of(-1).baseUnitMagnitude(), 0);
        assertEquals(100, baseUnit.of(100).baseUnitMagnitude(), 0);
        assertEquals(8.281723, baseUnit.of(8.281723).baseUnitMagnitude(), 0);
        assertEquals(Float.MAX_VALUE, baseUnit.of(Float.MAX_VALUE).baseUnitMagnitude(), 0);
        assertEquals(Float.MIN_VALUE, baseUnit.of(Float.MIN_VALUE).baseUnitMagnitude(), 0);
    }

    // Distances

    @Test
    void testMeters() {
        testBaseUnit(Meters);
        assertEquals("Meter", Meters.name());
        assertEquals("m", Meters.symbol());
    }

    @Test
    void testMillimeters() {
        assertEquals(1000, Millimeters.convertFrom(1, Meters), thresh);
        assertEquals(1, Meters.convertFrom(1000, Millimeters), thresh);
        assertEquals("Millimeter", Millimeters.name());
        assertEquals("mm", Millimeters.symbol());
    }

    @Test
    void testCentimeters() {
        assertEquals(100, Centimeters.convertFrom(1, Meters), thresh);
        assertEquals(1, Meters.convertFrom(100, Centimeters), thresh);
        assertEquals("Centimeter", Centimeters.name());
        assertEquals("cm", Centimeters.symbol());
    }

    @Test
    void testInches() {
        assertEquals(1, Meters.convertFrom(39.3701, Inches), thresh);
        assertEquals(39.3701, Inches.convertFrom(1, Meters), 1.0e-4);
        assertEquals(1 / 25.4, Inches.convertFrom(1, Millimeters), 0); // should be exact
        assertEquals(12, Inches.convertFrom(1, Feet), thresh);
        assertEquals("Inch", Inches.name());
        assertEquals("in", Inches.symbol());
    }

    @Test
    void testFeet() {
        assertEquals(3.28084, Feet.convertFrom(1, Meters), thresh);
        assertEquals(1 / 12.0, Feet.convertFrom(1, Inches), thresh);
        assertEquals("Foot", Feet.name());
        assertEquals("ft", Feet.symbol());
    }

    // Velocities

    @Test
    void testMetersPerSecond() {
        testBaseUnit(MetersPerSecond);
        assertEquals("Meter per Second", MetersPerSecond.name());
        assertEquals("m/s", MetersPerSecond.symbol());
    }

    @Test
    void testFeetPerSecond() {
        assertEquals(3.28084, FeetPerSecond.convertFrom(1, MetersPerSecond), thresh);
        assertEquals(1 / 3.28084, MetersPerSecond.convertFrom(1, FeetPerSecond), thresh);
        assertEquals("Foot per Second", FeetPerSecond.name());
        assertEquals("ft/s", FeetPerSecond.symbol());
    }

    // Accelerations

    @Test
    void testMetersPerSecondPerSecond() {
        testBaseUnit(MetersPerSecondPerSecond);
        assertEquals("Meter per Second per Second", MetersPerSecondPerSecond.name());
        assertEquals("m/s/s", MetersPerSecondPerSecond.symbol());
        assertEquals(MetersPerSecond, MetersPerSecondPerSecond.getUnit());
        assertEquals(Seconds, MetersPerSecondPerSecond.getPeriod());
    }

    @Test
    void testGs() {
        assertEquals(32.17405, Gs.of(1).in(FeetPerSecond.per(Second)), thresh);
        assertEquals(9.80665, Gs.of(1).in(MetersPerSecondPerSecond), thresh);
        assertEquals("G", Gs.name());
        assertEquals("G", Gs.symbol());
        assertEquals(MetersPerSecond, Gs.getUnit());
        assertEquals(Seconds, Gs.getPeriod());
    }

    // Time

    @Test
    void testSeconds() {
        testBaseUnit(Seconds);
        assertEquals("Second", Seconds.name());
        assertEquals("s", Seconds.symbol());
    }

    @Test
    void testMillisecond() {
        assertEquals(1000, Milliseconds.convertFrom(1, Seconds), thresh);
        assertEquals("Millisecond", Milliseconds.name());
        assertEquals("ms", Milliseconds.symbol());
    }

    @Test
    void testMicrosecond() {
        assertEquals(1.0e6, Microseconds.convertFrom(1, Second), 0);
        assertEquals("Microsecond", Microseconds.name());
        assertEquals("us", Microseconds.symbol());
    }

    @Test
    void testMinutes() {
        assertEquals(60, Seconds.convertFrom(1, Minutes), thresh);
        assertEquals("Minute", Minutes.name());
        assertEquals("min", Minutes.symbol());
    }

    // Mass

    @Test
    void testKilograms() {
        testBaseUnit(Kilograms);
        assertEquals("Kilogram", Kilograms.name());
        assertEquals("Kg", Kilograms.symbol());
    }

    @Test
    void testGrams() {
        assertEquals(1000, Grams.convertFrom(1, Kilograms), thresh);
        assertEquals("Gram", Grams.name());
        assertEquals("g", Grams.symbol());
    }

    @Test
    void testPounds() {
        assertEquals(453.592, Grams.convertFrom(1, Pounds), thresh);
        assertEquals("Pound", Pounds.name());
        assertEquals("lb.", Pounds.symbol());
    }

    @Test
    void testOunces() {
        assertEquals(16, Ounces.convertFrom(1, Pounds), thresh);
        assertEquals("Ounce", Ounces.name());
        assertEquals("oz.", Ounces.symbol());
    }

    // Angle

    @Test
    void testRevolutions() {
        assertEquals(1, Revolutions.convertFrom(Mathf.TWO_PI, Radians), thresh);
        assertEquals("Revolution", Revolutions.name());
        assertEquals("R", Revolutions.symbol());
    }

    @Test
    void testRadians() {
        testBaseUnit(Radians);
        assertEquals(Mathf.TWO_PI, Radians.convertFrom(1, Revolutions), thresh);
        assertEquals(Mathf.TWO_PI, Radians.convertFrom(360, Degrees), thresh);
        assertEquals("Radian", Radians.name());
        assertEquals("rad", Radians.symbol());
    }

    @Test
    void testDegrees() {
        assertEquals(360, Degrees.convertFrom(1, Revolutions), thresh);
        assertEquals(360, Degrees.convertFrom(Mathf.TWO_PI, Radians), thresh);
        assertEquals("Degree", Degrees.name());
        assertEquals("°", Degrees.symbol());
    }

    // Unitless

    @Test
    void testValue() {
        testBaseUnit(Value);
        assertEquals("<?>", Value.name());
        assertEquals("<?>", Value.symbol());
    }

    @Test
    void testPercent() {
        assertEquals(100, Percent.convertFrom(1, Value), thresh);
        assertEquals("Percent", Percent.name());
        assertEquals("%", Percent.symbol());
    }

    // Electric potential

    @Test
    void testVolts() {
        testBaseUnit(Volts);
        assertEquals("Volt", Volts.name());
        assertEquals("V", Volts.symbol());
    }

    @Test
    void testMillivolts() {
        assertEquals(1000, Millivolts.convertFrom(1, Volts), thresh);
        assertEquals("Millivolt", Millivolts.name());
        assertEquals("mV", Millivolts.symbol());
    }

    // Electric current

    @Test
    void testAmps() {
        testBaseUnit(Amps);
        assertEquals("Amp", Amps.name());
        assertEquals("A", Amps.symbol());
    }

    @Test
    void testMilliamps() {
        assertEquals(1000, Milliamps.convertFrom(1, Amps), thresh);
        assertEquals("Milliamp", Milliamps.name());
        assertEquals("mA", Milliamps.symbol());
    }

    // Power

    @Test
    void testWatts() {
        testBaseUnit(Watts);
        assertEquals("Watt", Watts.name());
        assertEquals("W", Watts.symbol());
    }

    @Test
    void testMilliwatts() {
        assertEquals(1000, Milliwatts.convertFrom(1, Watts), thresh);
        assertEquals("Milliwatt", Milliwatts.name());
        assertEquals("mW", Milliwatts.symbol());
    }

    @Test
    void testHorsepower() {
        assertEquals(745.7, Watts.convertFrom(1, Horsepower), thresh);
        assertEquals("Horsepower", Horsepower.name());
        assertEquals("HP", Horsepower.symbol());
    }

    // Temperature

    @Test
    void testKelvin() {
        testBaseUnit(Kelvin);
        assertEquals("Kelvin", Kelvin.name());
        assertEquals("K", Kelvin.symbol()); // note: there's no degree symbol for Kelvin!
    }

    @Test
    void testCelsius() {
        assertEquals(0, Celsius.of(-273.15).in(Kelvin), thresh);
        assertEquals(273.15, Celsius.of(0).in(Kelvin), thresh);
        assertEquals(0, Kelvin.of(273.15).in(Celsius), thresh);
        assertTrue(Celsius.of(0).isEquivalent(Kelvin.of(273.15)));
        assertTrue(Celsius.of(-273.15).isEquivalent(Kelvin.of(0)));
        assertEquals("Celsius", Celsius.name());
        assertEquals("°C", Celsius.symbol());
    }

    @Test
    void testFahrenheit() {
        assertEquals(0, Fahrenheit.of(32).in(Celsius), thresh);
        assertEquals(100, Fahrenheit.of(212).in(Celsius), thresh);
        assertEquals(-459.67, Kelvin.of(0).in(Fahrenheit), thresh);
        assertEquals(273.15, Fahrenheit.of(32).in(Kelvin), thresh);
        assertEquals(32, Kelvin.of(273.15).in(Fahrenheit), thresh);
        assertTrue(Fahrenheit.of(32).isEquivalent(Celsius.of(0)));
        assertTrue(Fahrenheit.of(212).isEquivalent(Celsius.of(100)));
        assertEquals("Fahrenheit", Fahrenheit.name());
        assertEquals("°F", Fahrenheit.symbol());
    }

    // Helpers

    @Test
    void testKilo() {
        ExampleUnit unit = new ExampleUnit(1);
        ExampleUnit kiloUnit = Kilo(unit);
        assertEquals(1.0e3, unit.convertFrom(1, kiloUnit), 0);
        assertEquals(1.0e-3, kiloUnit.convertFrom(1, unit), 0);
    }

    @Test
    void testMilli() {
        ExampleUnit unit = new ExampleUnit(1);
        ExampleUnit milliUnit = Milli(unit);
        assertEquals(1.0e-3, unit.convertFrom(1, milliUnit), 0);
        assertEquals(1.0e3, milliUnit.convertFrom(1, unit), 0);
    }
}