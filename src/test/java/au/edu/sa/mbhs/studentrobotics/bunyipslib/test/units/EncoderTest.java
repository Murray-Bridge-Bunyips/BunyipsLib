package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Revolutions;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.MutableMeasure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Unit;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

class EncoderTest {
    @Test
    void testAsDistance() {
        double ticksPerRevolution = 2048;

        Encoder<Distance> encoder = new Encoder<>();

        // distance per rotation = (wheel circumference / gear ratio)
        // distance per tick = distance per rotation / ticks per rotation
        Measure<Distance> wheelDiameter = Inches.of(6);
        double gearRatio = 10; // 10:1 ratio
        Measure<Distance> distancePerPulse =
                wheelDiameter.times(Math.PI).div(gearRatio).div(ticksPerRevolution);
        encoder.setDistancePerPulse(distancePerPulse);

        encoder.ticks = 0;
        assertEquals(0, encoder.getDistance().in(Inches), Measure.EQUIVALENCE_THRESHOLD);
        assertEquals(0, encoder.getRate().in(Inches.per(Second)), Measure.EQUIVALENCE_THRESHOLD);

        // one full encoder turn, 1/10th of a wheel rotation
        encoder.setTicks(2048);
        assertEquals(6 * Math.PI / 10, encoder.getDistance().in(Inches), Measure.EQUIVALENCE_THRESHOLD);

        // one full encoder turn back, 1/10th of a wheel rotation - rate should be negative
        encoder.setTicks(0);
        assertEquals(-6 * Math.PI / 10, encoder.getRate().in(Inches.per(Second)), Measure.EQUIVALENCE_THRESHOLD);
    }

    @Test
    void testAsRevolutions() {
        double ticksPerRevolution = 2048;

        Encoder<Angle> encoder = new Encoder<>();

        Measure<Angle> distancePerPulse = Revolutions.of(1).div(ticksPerRevolution);
        encoder.setDistancePerPulse(distancePerPulse);

        encoder.ticks = 0;
        assertEquals(0, encoder.getDistance().in(Revolutions), Measure.EQUIVALENCE_THRESHOLD);
        assertEquals(0, encoder.getRate().in(Revolutions.per(Second)), Measure.EQUIVALENCE_THRESHOLD);

        encoder.setTicks(2048); // one full encoder turn, 1/10th of a wheel rotation
        assertEquals(1, encoder.getDistance().in(Revolutions), Measure.EQUIVALENCE_THRESHOLD);
        assertEquals(1, encoder.getRate().in(Revolutions.per(Second)), Measure.EQUIVALENCE_THRESHOLD);
    }

    static class Encoder<U extends Unit<U>> {
        int ticks;
        private Measure<U> distancePerPulse;
        private MutableMeasure<U> distance;
        private MutableMeasure<Velocity<U>> rate;

        void setDistancePerPulse(Measure<U> distancePerPulse) {
            this.distancePerPulse = distancePerPulse;
            distance = MutableMeasure.zero(distancePerPulse.unit());
            rate = MutableMeasure.zero(distancePerPulse.unit().per(Second));
        }

        Measure<U> getDistance() {
            return distance;
        }

        Measure<Velocity<U>> getRate() {
            return rate;
        }

        void setTicks(int ticks) {
            // pretend we read from JNI here instead of being passed a specific value
            double change = ticks - this.ticks;
            this.ticks = ticks;
            distance.mut_setMagnitude(distancePerPulse.magnitude() * ticks);

            // assumes the last update was 1 second ago - fine for tests
            rate.mut_setMagnitude(distancePerPulse.magnitude() * change);
        }
    }
}