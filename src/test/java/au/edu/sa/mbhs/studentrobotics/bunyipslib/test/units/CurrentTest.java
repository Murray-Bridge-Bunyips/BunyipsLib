package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Power;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units;

class CurrentTest {
    @Test
    void testAmpsTimesVolts() {
        Power combined = Units.Amps.times(Units.Volts, "Watt", "w");
        assertTrue(combined.equivalent(Units.Watts));
    }

    @Test
    void testMilliAmpsTimesMilliVolts() {
        // results in microwatts
        assertTrue(Units.Milliamps.times(Units.Millivolts, "Microwatt", "uW")
                .equivalent(Units.Milli(Units.Milliwatts)));
    }
}