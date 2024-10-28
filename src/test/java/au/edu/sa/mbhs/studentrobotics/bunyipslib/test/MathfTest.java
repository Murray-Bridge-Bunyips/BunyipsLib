package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import org.apache.commons.math3.util.MathUtils;
import org.junit.jupiter.api.Test;

import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Reference;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;

/**
 * Various Mathf utility tests.
 *
 * @author Lucas Bubner, 2024
 */
class MathfTest {
    @Test
    void testRounding() {
        assertEquals(1, Mathf.round(0.5, 0));
        assertEquals(-12.24, Mathf.round(-12.239999, 2));
        assertEquals(0.556, Mathf.round(0.5564, 3));
        assertEquals(18.4446, Mathf.round(18.44455, 4));
        assertEquals(1.234567891234568, Mathf.round(1.23456789123456789, 15));
    }

    @Test
    void testSigFigRounding() {
        assertEquals(123.46, Mathf.round(123.456789, 3, 5));
        assertEquals(123.457, Mathf.round(123.456789, 4, 6));
        assertEquals(123.4568, Mathf.round(123.456789, 5, 7));
        assertEquals(0.12, Mathf.round(0.123456, 2, 4));
        assertEquals(-0.123, Mathf.round(-0.123456, 3, 4));
        assertEquals(987.654, Mathf.round(987.654321, 3, 6));
        assertEquals(-987.654, Mathf.round(-987.654321, 3, 6));
        assertEquals(-123.46, Mathf.round(-123.456789, 3, 5));
        assertEquals(-123.457, Mathf.round(-123.456789, 4, 6));
        assertEquals(-123.4568, Mathf.round(-123.456789, 5, 7));
        assertEquals(-123.45679, Mathf.round(-123.456789, 6, 8));
        assertEquals(-123.456789, Mathf.round(-123.456789, 7, 9));
    }

    @Test
    void testApprox() {
        assertTrue(Mathf.approx(1.0000001, 1.0000002));
        assertFalse(Mathf.approx(1.00001, 1.00002));
        assertTrue(Mathf.approx(0.0, -0.0));
        assertFalse(Mathf.approx(Double.MAX_VALUE, Double.MIN_VALUE));
    }

    @Test
    void testClamp() {
        assertEquals(5.0, Mathf.clamp(10.0, 0.0, 5.0));
        assertEquals(0.0, Mathf.clamp(-10.0, 0.0, 5.0));
        assertEquals(3.0, Mathf.clamp(3.0, 0.0, 5.0));
        assertEquals(0.0, Mathf.clamp(0.0, 0.0, 5.0));
        assertEquals(5.0, Mathf.clamp(5.0, 0.0, 5.0));
        assertEquals(2.5, Mathf.clamp(2.5, 0.0, 5.0));
        assertEquals(5, Mathf.clamp(10, 1, 5));
        assertEquals(5.5, Mathf.clamp(10.5, 1.5, 5.5));
        assertEquals(-5, Mathf.clamp(-10, -5, -1));
        assertEquals(-5.5, Mathf.clamp(-10.5, -5.5, -1.5));
    }

    @Test
    void testWrap() {
        assertEquals(1.0, Mathf.wrap(361.0, 0.0, 360.0));
        assertEquals(359.0, Mathf.wrap(-1.0, 0.0, 360.0));
        assertEquals(360.0, Mathf.wrap(360.0, 0.0, 360.0));
        assertEquals(180.0, Mathf.wrap(540.0, 0.0, 360.0));
        assertEquals(180.0, Mathf.wrap(-180.0, 0.0, 360.0));
        assertEquals(0.0, Mathf.wrap(Radians.of(MathUtils.TWO_PI)).in(Radians));
        assertEquals(0.0, Mathf.wrap(Radians.of(-MathUtils.TWO_PI)).in(Radians));
        assertEquals(0.0, Mathf.wrapRadians(MathUtils.TWO_PI));
        assertEquals(0.0, Mathf.wrapRadians(-MathUtils.TWO_PI));
        assertEquals(Mathf.wrapDeltaRadians(Math.toRadians(-2000)), Math.toRadians(160), 1.0e-6);
        assertEquals(Mathf.wrapDeltaRadians(Math.toRadians(358)), Math.toRadians(-2), 1.0e-6);
        assertEquals(Mathf.wrapDeltaRadians(Math.toRadians(360)), 0, 1.0e-6);
        assertEquals(Mathf.wrapDeltaRadians(5 * Math.PI), Math.PI);
        assertEquals(Mathf.wrapDeltaRadians(-5 * Math.PI), Math.PI);
        assertEquals(Mathf.wrapDeltaRadians(Math.PI / 2), Math.PI / 2);
        assertEquals(Mathf.wrapDeltaRadians(-Math.PI / 2), -Math.PI / 2);
    }

    @Test
    void testLerp() {
        assertEquals(5.0, Mathf.lerp(0.0, 10.0, 0.5));
        assertEquals(10.0, Mathf.lerp(0.0, 10.0, 1.0));
        assertEquals(0.0, Mathf.lerp(0.0, 10.0, 0.0));
        assertEquals(-5.0, Mathf.lerp(-10.0, 0.0, 0.5));
        assertEquals(15.0, Mathf.lerp(10.0, 20.0, 0.5));
        assertEquals(50, Mathf.lerp(0, 100, 0.5));
        assertEquals(-50, Mathf.lerp(0, -100, 0.5));
        assertEquals(0, Mathf.lerp(-50, 50, 0.5));
        assertEquals(-25, Mathf.lerp(-50, 50, 0.25));
        assertEquals(25, Mathf.lerp(-50, 50, 0.75));
        assertEquals(0, Mathf.lerp(0, -100, -0.5));
    }

    @Test
    void testLerpUnclamped() {
        assertEquals(-5.0, Mathf.lerpUnclamped(0.0, 10.0, -0.5));
        assertEquals(20.0, Mathf.lerpUnclamped(0.0, 10.0, 2.0));
        assertEquals(-10.0, Mathf.lerpUnclamped(0.0, 10.0, -1.0));
        assertEquals(-15.0, Mathf.lerpUnclamped(-10.0, 0.0, -0.5));
        assertEquals(25.0, Mathf.lerpUnclamped(10.0, 20.0, 1.5));
        assertEquals(150, Mathf.lerpUnclamped(0, 100, 1.5));
        assertEquals(-150, Mathf.lerpUnclamped(0, -100, 1.5));
        assertEquals(-100, Mathf.lerpUnclamped(-50, 50, -0.5));
        assertEquals(-75, Mathf.lerpUnclamped(-50, 50, -0.25));
        assertEquals(75, Mathf.lerpUnclamped(-50, 50, 1.25));
        assertEquals(50, Mathf.lerpUnclamped(0, -100, -0.5));
    }

    @Test
    void testInverseLerp() {
        assertEquals(0.5, Mathf.inverseLerp(0.0, 10.0, 5.0));
        assertEquals(1.0, Mathf.inverseLerp(0.0, 10.0, 10.0));
        assertEquals(0.0, Mathf.inverseLerp(0.0, 10.0, 0.0));
        assertEquals(0.25, Mathf.inverseLerp(0.0, 20.0, 5.0));
        assertEquals(0.75, Mathf.inverseLerp(0.0, 20.0, 15.0));
    }

    @Test
    void testSmoothStep() {
        assertEquals(0.0, Mathf.smoothStep(0.0, 1.0, 0.0), 1.0e-6);
        assertEquals(0.0, Mathf.smoothStep(0.0, 1.0, -0.1), 1.0e-6);
        assertEquals(0.028, Mathf.smoothStep(0.0, 1.0, 0.1), 1.0e-6);
        assertEquals(1.0, Mathf.smoothStep(0.0, 1.0, 1.0), 1.0e-6);
        assertEquals(1.0, Mathf.smoothStep(0.0, 1.0, 1.1), 1.0e-6);
        assertEquals(0.972, Mathf.smoothStep(0.0, 1.0, 0.9), 1.0e-6);
        assertEquals(0.5, Mathf.smoothStep(0.0, 1.0, 0.5), 1.0e-6);
        assertEquals(1.0, Mathf.smoothStep(0.0, 1.0, 1.0), 1.0e-6);
        assertEquals(0.0, Mathf.smoothStep(0.0, 1.0, 0.0), 1.0e-6);
        assertEquals(0.15625, Mathf.smoothStep(0.0, 1.0, 0.25), 1.0e-6);
        assertEquals(0.84375, Mathf.smoothStep(0.0, 1.0, 0.75), 1.0e-6);
    }

    @Test
    void testSolveQuadratic() {
        List<Double> roots = Mathf.solveQuadratic(1.0, -3.0, 2.0);
        assertEquals(2, roots.size());
        assertEquals(2.0, roots.get(0));
        assertEquals(1.0, roots.get(1));

        roots = Mathf.solveQuadratic(1.0, -2.0, 1.0);
        assertEquals(1, roots.size());
        assertEquals(1.0, roots.get(0));

        roots = Mathf.solveQuadratic(1.0, 0.0, -1.0);
        assertEquals(2, roots.size());
        assertEquals(1.0, roots.get(0));
        assertEquals(-1.0, roots.get(1));

        roots = Mathf.solveQuadratic(1.0, 0.0, 1.0);
        assertEquals(0, roots.size());

        roots = Mathf.solveQuadratic(23.0, -48.0, 1.0);
        assertEquals(2, roots.size());
        assertEquals(2.06591, roots.get(0), 1.0e-5);
        assertEquals(0.02105, roots.get(1), 1.0e-5);

        roots = Mathf.solveQuadratic(238442, -48445134, -21321);
        assertEquals(2, roots.size());
        assertEquals(203.174100786, roots.get(0), 1.0e-9);
        assertEquals(-0.000440105167547, roots.get(1), 1.0e-9);

        roots = Mathf.solveQuadratic(1, 0, 0);
        assertEquals(1, roots.size());
    }

    @Test
    void testScale() {
        assertEquals(0.0, Mathf.scale(0.0, 0.0, 1.0, 0.0, 1.0));
        assertEquals(1.0, Mathf.scale(1.0, 0.0, 1.0, 0.0, 1.0));
        assertEquals(0.5, Mathf.scale(0.5, 0.0, 1.0, 0.0, 1.0));
        assertEquals(0.0, Mathf.scale(0.0, 0.0, 1.0, 0.0, 10.0));
        assertEquals(10.0, Mathf.scale(1.0, 0.0, 1.0, 0.0, 10.0));
        assertEquals(5.0, Mathf.scale(0.5, 0.0, 1.0, 0.0, 10.0));
        assertEquals(0.0, Mathf.scale(0.0, 0.0, 10.0, 0.0, 1.0));
        assertEquals(1.0, Mathf.scale(10.0, 0.0, 10.0, 0.0, 1.0));
        assertEquals(0.5, Mathf.scale(5.0, 0.0, 10.0, 0.0, 1.0));
        assertEquals(0.0, Mathf.scale(0.0, 0.0, 10.0, 0.0, 10.0));
        assertEquals(10.0, Mathf.scale(10.0, 0.0, 10.0, 0.0, 10.0));
        assertEquals(5.0, Mathf.scale(5.0, 0.0, 10.0, 0.0, 10.0));
    }

    @Test
    void testApplyDeadbandUnityScale() {
        // < 0
        assertEquals(-1.0, Mathf.applyDeadband(-1.0, 0.02));
        assertEquals((-0.03 + 0.02) / (1.0 - 0.02), Mathf.applyDeadband(-0.03, 0.02));
        assertEquals(0.0, Mathf.applyDeadband(-0.02, 0.02));
        assertEquals(0.0, Mathf.applyDeadband(-0.01, 0.02));

        // == 0
        assertEquals(0.0, Mathf.applyDeadband(0.0, 0.02));

        // > 0
        assertEquals(0.0, Mathf.applyDeadband(0.01, 0.02));
        assertEquals(0.0, Mathf.applyDeadband(0.02, 0.02));
        assertEquals((0.03 - 0.02) / (1.0 - 0.02), Mathf.applyDeadband(0.03, 0.02));
        assertEquals(1.0, Mathf.applyDeadband(1.0, 0.02));
    }

    @Test
    void testApplyDeadbandArbitraryScale() {
        // < 0
        assertEquals(-2.5, Mathf.applyDeadband(-2.5, 0.02, 2.5));
        assertEquals(0.0, Mathf.applyDeadband(-0.02, 0.02, 2.5));
        assertEquals(0.0, Mathf.applyDeadband(-0.01, 0.02, 2.5));

        // == 0
        assertEquals(0.0, Mathf.applyDeadband(0.0, 0.02, 2.5));

        // > 0
        assertEquals(0.0, Mathf.applyDeadband(0.01, 0.02, 2.5));
        assertEquals(0.0, Mathf.applyDeadband(0.02, 0.02, 2.5));
        assertEquals(2.5, Mathf.applyDeadband(2.5, 0.02, 2.5));
    }

    @Test
    void testApplyDeadbandLargeMaxMagnitude() {
        assertEquals(80.0, Mathf.applyDeadband(100.0, 20, Double.POSITIVE_INFINITY));
    }

    @Test
    void testInputModulus() {
        // These tests check error wrapping. That is, the result of wrapping the
        // result of an angle reference minus the measurement.

        // Test symmetric range
        assertEquals(-20.0, Mathf.wrap(170.0 - (-170.0), -180.0, 180.0));
        assertEquals(-20.0, Mathf.wrap(170.0 + 360.0 - (-170.0), -180.0, 180.0));
        assertEquals(-20.0, Mathf.wrap(170.0 - (-170.0 + 360.0), -180.0, 180.0));
        assertEquals(20.0, Mathf.wrap(-170.0 - 170.0, -180.0, 180.0));
        assertEquals(20.0, Mathf.wrap(-170.0 + 360.0 - 170.0, -180.0, 180.0));
        assertEquals(20.0, Mathf.wrap(-170.0 - (170.0 + 360.0), -180.0, 180.0));

        // Test range start at zero
        assertEquals(340.0, Mathf.wrap(170.0 - 190.0, 0.0, 360.0));
        assertEquals(340.0, Mathf.wrap(170.0 + 360.0 - 190.0, 0.0, 360.0));
        assertEquals(340.0, Mathf.wrap(170.0 - (190.0 + 360), 0.0, 360.0));

        // Test asymmetric range that doesn't start at zero
        assertEquals(-20.0, Mathf.wrap(170.0 - (-170.0), -170.0, 190.0));

        // Test range with both positive endpoints
        assertEquals(2.0, Mathf.wrap(0.0, 1.0, 3.0));
        assertEquals(3.0, Mathf.wrap(1.0, 1.0, 3.0));
        assertEquals(2.0, Mathf.wrap(2.0, 1.0, 3.0));
        assertEquals(3.0, Mathf.wrap(3.0, 1.0, 3.0));
        assertEquals(2.0, Mathf.wrap(4.0, 1.0, 3.0));
    }

    @Test
    void testIsNear() {
        // The answer is always 42
        // Positive integer checks
        assertTrue(Mathf.isNear(42, 42, 1));
        assertTrue(Mathf.isNear(42, 41, 2));
        assertTrue(Mathf.isNear(42, 43, 2));
        assertFalse(Mathf.isNear(42, 44, 1));

        // Negative integer checks
        assertTrue(Mathf.isNear(-42, -42, 1));
        assertTrue(Mathf.isNear(-42, -41, 2));
        assertTrue(Mathf.isNear(-42, -43, 2));
        assertFalse(Mathf.isNear(-42, -44, 1));

        // Mixed sign integer checks
        assertFalse(Mathf.isNear(-42, 42, 1));
        assertFalse(Mathf.isNear(-42, 41, 2));
        assertFalse(Mathf.isNear(-42, 43, 2));
        assertFalse(Mathf.isNear(42, -42, 1));
        assertFalse(Mathf.isNear(42, -41, 2));
        assertFalse(Mathf.isNear(42, -43, 2));

        // Floating point checks
        assertTrue(Mathf.isNear(42, 41.5, 1));
        assertTrue(Mathf.isNear(42, 42.5, 1));
        assertTrue(Mathf.isNear(42, 41.5, 0.75));
        assertTrue(Mathf.isNear(42, 42.5, 0.75));

        // Wraparound checks
        assertTrue(Mathf.isNear(0, 356, 5, 0, 360));
        assertTrue(Mathf.isNear(0, -356, 5, 0, 360));
        assertTrue(Mathf.isNear(0, 4, 5, 0, 360));
        assertTrue(Mathf.isNear(0, -4, 5, 0, 360));
        assertTrue(Mathf.isNear(400, 41, 5, 0, 360));
        assertTrue(Mathf.isNear(400, -319, 5, 0, 360));
        assertTrue(Mathf.isNear(400, 401, 5, 0, 360));
        assertFalse(Mathf.isNear(0, 356, 2.5, 0, 360));
        assertFalse(Mathf.isNear(0, -356, 2.5, 0, 360));
        assertFalse(Mathf.isNear(0, 4, 2.5, 0, 360));
        assertFalse(Mathf.isNear(0, -4, 2.5, 0, 360));
        assertFalse(Mathf.isNear(400, 35, 5, 0, 360));
        assertFalse(Mathf.isNear(400, -315, 5, 0, 360));
        assertFalse(Mathf.isNear(400, 395, 5, 0, 360));
    }

    @Test
    void testMoveTowards() {
        // Target within maxDelta
        assertEquals(5.0, Mathf.moveTowards(3.0, 5.0, 3.0), 1.0e-6);
        assertEquals(-5.0, Mathf.moveTowards(-3.0, -5.0, 3.0), 1.0e-6);
        // Target beyond maxDelta
        assertEquals(4.0, Mathf.moveTowards(3.0, 10.0, 1.0), 1.0e-6);
        assertEquals(-4.0, Mathf.moveTowards(-3.0, -10.0, 1.0), 1.0e-6);
        // Target with zero maxDelta
        assertEquals(3.0, Mathf.moveTowards(3.0, 10.0, 0.0), 1.0e-6);
        assertEquals(-3.0, Mathf.moveTowards(-3.0, -10.0, 0.0), 1.0e-6);
        // Target with negative maxDelta
        assertEquals(2.0, Mathf.moveTowards(3.0, 10.0, -1.0), 1.0e-6);
        assertEquals(-4.0, Mathf.moveTowards(-3.0, -10.0, -1.0), 1.0e-6);
        // maxDelta larger than the difference
        assertEquals(10.0, Mathf.moveTowards(3.0, 10.0, 10.0), 1.0e-6);
        assertEquals(-10.0, Mathf.moveTowards(-3.0, -10.0, 10.0), 1.0e-6);
    }

//    @Test TODO
    void testMoveTowardsAngle() {
        // Target within maxDelta
        assertEquals(Degrees.of(45.0), Mathf.moveTowards(Degrees.of(30.0), Degrees.of(45.0), Degrees.of(20.0)));
        assertEquals(Degrees.of(-45.0), Mathf.moveTowards(Degrees.of(-30.0), Degrees.of(-45.0), Degrees.of(20.0)));
        // Target beyond maxDelta
        assertEquals(Degrees.of(35.0), Mathf.moveTowards(Degrees.of(30.0), Degrees.of(60.0), Degrees.of(5.0)));
        assertEquals(Degrees.of(-35.0), Mathf.moveTowards(Degrees.of(-30.0), Degrees.of(-60.0), Degrees.of(5.0)));
        // Target with zero maxDelta
        assertEquals(Degrees.of(30.0), Mathf.moveTowards(Degrees.of(30.0), Degrees.of(60.0), Degrees.of(0.0)));
        assertEquals(Degrees.of(-30.0), Mathf.moveTowards(Degrees.of(-30.0), Degrees.of(-60.0), Degrees.of(0.0)));
        // Target with negative maxDelta (should return current value)
        assertEquals(Degrees.of(30.0), Mathf.moveTowards(Degrees.of(30.0), Degrees.of(60.0), Degrees.of(-5.0)));
        assertEquals(Degrees.of(-30.0), Mathf.moveTowards(Degrees.of(-30.0), Degrees.of(-60.0), Degrees.of(-5.0)));
        // maxDelta larger than the difference
        assertEquals(Degrees.of(60.0), Mathf.moveTowards(Degrees.of(30.0), Degrees.of(60.0), Degrees.of(40.0)));
        assertEquals(Degrees.of(-60.0), Mathf.moveTowards(Degrees.of(-30.0), Degrees.of(-60.0), Degrees.of(40.0)));
        // Wrapping around 360 degrees
        assertEquals(Degrees.of(350.0), Mathf.moveTowards(Degrees.of(10.0), Degrees.of(350.0), Degrees.of(20.0)));
        assertEquals(Degrees.of(10.0), Mathf.moveTowards(Degrees.of(350.0), Degrees.of(10.0), Degrees.of(20.0)));
        assertEquals(Degrees.of(5.0), Mathf.moveTowards(Degrees.of(355.0), Degrees.of(5.0), Degrees.of(10.0)));
        assertEquals(Degrees.of(355.0), Mathf.moveTowards(Degrees.of(5.0), Degrees.of(355.0), Degrees.of(10.0)));
    }

    @Test
    void testGamma() {
        assertEquals(0.0, Mathf.gamma(0.0, 1.0, 2.0), 1.0e-6);
        assertEquals(0.25, Mathf.gamma(0.5, 1.0, 2.0), 1.0e-6);
        assertEquals(1.0, Mathf.gamma(1.0, 1.0, 2.0), 1.0e-6);
        assertEquals(0.0, Mathf.gamma(-0.0, 1.0, 2.0), 1.0e-6);
        assertEquals(-0.25, Mathf.gamma(-0.5, 1.0, 2.0), 1.0e-6);
        assertEquals(-1.0, Mathf.gamma(-1.0, 1.0, 2.0), 1.0e-6);
        assertEquals(1.5, Mathf.gamma(1.5, 1.0, 2.0), 1.0e-6);
        assertEquals(-1.5, Mathf.gamma(-1.5, 1.0, 2.0), 1.0e-6);
        assertEquals(0.5, Mathf.gamma(0.5, 1.0, 1.0), 1.0e-6);
        assertEquals(0.125, Mathf.gamma(0.5, 1.0, 3.0), 1.0e-6);
        assertEquals(0.70710678118, Mathf.gamma(0.5, 1.0, 0.5), 1.0e-6);
    }

    @Test
    void testSmoothDamp() {
        Reference<Double> velocity = new Reference<>(0.0);

        // Test moving towards target
        assertEquals(2.79411764, Mathf.smoothDamp(0.0, 5.0, velocity, Seconds.of(1.0), 10.0, Seconds.of(1.0)), 1.0e-6);
        assertEquals(velocity.require(), 2.94117647, 1.0e-6);
        assertEquals(8.22664359, Mathf.smoothDamp(5.0, 10.0, velocity, Seconds.of(1.0), 10.0, Seconds.of(1.0)), 1.0e-6);

        velocity.set(0.0);
        Mathf.smoothDamp(0.0, 1000.0, velocity, Seconds.of(100.0), 1.0, Seconds.of(50.0));
        assertEquals(0.73664, velocity.require(), 1.0e-3);

        // Test overshooting prevention
        velocity.set(0.0);
        assertEquals(9.5588235, Mathf.smoothDamp(9.0, 10.0, velocity, Seconds.of(1.0), 10.0, Seconds.of(1.0)), 1.0e-6);

        // Test clamping of maximum speed
        velocity.set(0.0);
        assertEquals(0.5588235, Mathf.smoothDamp(0.0, 10.0, velocity, Seconds.of(1.0), 1.0, Seconds.of(1.0)), 1.0e-6);

        // Test negative target
        velocity.set(0.0);
        assertEquals(-2.794117, Mathf.smoothDamp(0.0, -5.0, velocity, Seconds.of(1.0), 10.0, Seconds.of(1.0)), 1.0e-6);
        assertEquals(-8.226643, Mathf.smoothDamp(-5.0, -10.0, velocity, Seconds.of(1.0), 10.0, Seconds.of(1.0)), 1.0e-6);

        // Test with different smoothTime and deltaTime
        velocity.set(0.0);
        assertEquals(1.3167587, Mathf.smoothDamp(0.0, 5.0, velocity, Seconds.of(2.0), 10.0, Seconds.of(1.0)), 1.0e-6);
        assertEquals(3.8366926, Mathf.smoothDamp(2.5, 5.0, velocity, Seconds.of(2.0), 10.0, Seconds.of(1.0)), 1.0e-6);
    }
}
