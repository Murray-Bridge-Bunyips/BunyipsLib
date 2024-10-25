package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Amps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milli;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliamps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Millivolts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliwatts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Volts;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Watts;

import org.junit.jupiter.api.Test;

class VoltageTest {
    @Test
    void testVoltsTimesAmps() {
        assertTrue(Volts.times(Amps, "", "").equivalent(Watts));
    }

    @Test
    void testMilliVoltsTimesMilliAmps() {
        // results in microwatts
        assertTrue(Millivolts.times(Milliamps, "", "").equivalent(Milli(Milliwatts)));
    }
}