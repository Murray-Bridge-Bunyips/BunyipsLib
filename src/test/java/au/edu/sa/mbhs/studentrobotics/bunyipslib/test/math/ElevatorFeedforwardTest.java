package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.ElevatorFeedforward;

class ElevatorFeedforwardTest {
    private static final double ks = 0.5;
    private static final double kg = 1;
    private static final double kv = 1.5;
    private static final double ka = 2;

    private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(ks, kg, kv, ka);

    @Test
    void testCalculate() {
        assertEquals(1, elevatorFF.calculate(0), 0.002);
        assertEquals(4.5, elevatorFF.calculate(2), 0.002);
    }

    @Test
    void testAchievableVelocity() {
        assertEquals(5, elevatorFF.maxAchievableVelocity(11, 1), 0.002);
        assertEquals(-9, elevatorFF.minAchievableVelocity(11, 1), 0.002);
    }

    @Test
    void testAchievableAcceleration() {
        assertEquals(3.75, elevatorFF.maxAchievableAcceleration(12, 2), 0.002);
        assertEquals(7.25, elevatorFF.maxAchievableAcceleration(12, -2), 0.002);
        assertEquals(-8.25, elevatorFF.minAchievableAcceleration(12, 2), 0.002);
        assertEquals(-4.75, elevatorFF.minAchievableAcceleration(12, -2), 0.002);
    }
}
