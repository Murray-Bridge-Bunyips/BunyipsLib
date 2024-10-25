package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.lang.reflect.Constructor;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

class DistanceTest {
    @Test
    void testBaseUnitDistancePerTime() throws Exception {
        Constructor<Distance> distanceConstructor = Distance.class.getDeclaredConstructor(Distance.class, double.class, String.class, String.class);
        distanceConstructor.setAccessible(true);
        Distance distance = distanceConstructor.newInstance(null, 1, "D", "d");

        Constructor<Time> timeConstructor = Time.class.getDeclaredConstructor(Time.class, double.class, String.class, String.class);
        timeConstructor.setAccessible(true);
        Time time = timeConstructor.newInstance(null, 1, "T", "t");

        Velocity<Distance> anonBaseUnit = distance.per(time);
        assertTrue(Units.MetersPerSecond.equivalent(anonBaseUnit));
    }

    @Test
    void testFeetPerSecond() {
        Velocity<Distance> feetPerMillisecond = Units.Feet.per(Units.Milliseconds);

        // one foot per millisecond
        // = (1 / 3.28084) meters per (1 / 1000) seconds
        // = (1000 / 3.28084) meters per second
        double asBaseMeasure = feetPerMillisecond.of(1).in(Units.MetersPerSecond);
        assertEquals(1000 / 3.28084, asBaseMeasure, 1.0e-3);

        // one meter per second = 1 mm per millisecond = 0.00328084 feet per millisecond
        double asContrivedMeasure = Units.MetersPerSecond.of(1).in(feetPerMillisecond);
        assertEquals(3.28084 / 1000, asContrivedMeasure, 1.0e-8);
    }
}