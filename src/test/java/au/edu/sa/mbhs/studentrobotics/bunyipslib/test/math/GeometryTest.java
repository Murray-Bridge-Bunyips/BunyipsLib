package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.FieldTiles;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Reference;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Geometry class tests.
 *
 * @author Lucas Bubner, 2024
 */
class GeometryTest {
    @Test
    void testZeroPose() {
        assertEquals(new Pose2d(0, 0, 0), Geometry.zeroPose());
    }

    @Test
    void testZeroVel() {
        assertEquals(new PoseVelocity2d(new Vector2d(0, 0), 0), Geometry.zeroVel());
    }

    @Test
    void testZeroVec() {
        assertEquals(new Vector2d(0, 0), Geometry.zeroVec());
    }

    @Test
    void testZeroTwist() {
        assertEquals(new Twist2d(new Vector2d(0, 0), 0), Geometry.zeroTwist());
    }

    @Test
    void testVel() {
        assertEquals(new PoseVelocity2d(new Vector2d(12, 34), 56), Geometry.vel(12, 34, 56));
    }

    @Test
    void testInchesFrom() {
        Vector2d a = new Vector2d(24, 24);
        Vector2d b = Geometry.inchesFrom(new Vector2d(1, 1), FieldTiles);
        assertEquals(a.x, b.x, 1.0e-6);
        assertEquals(a.y, b.y, 1.0e-6);
    }

    @Test
    void testPoseFrom() {
        Pose2d a = new Pose2d(24, 24, Math.PI / 2);
        Pose2d b = Geometry.poseFrom(new Vector2d(1, 1), FieldTiles, 90, Degrees);
        assertEquals(a.position.x, b.position.x, 1.0e-6);
        assertEquals(a.position.y, b.position.y, 1.0e-6);
        assertEquals(a.heading.log(), b.heading.log(), 1.0e-6);
    }

    @Test
    void testApproxPose() {
        Pose2d a = new Pose2d(24, 24, Math.PI / 2);
        Pose2d b = new Pose2d(24.0000001, 24.0000001, Math.PI / 2 + 0.000001);

        assertTrue(Geometry.approx(a, b));
    }

    @Test
    void testApproxPoseNormHeading() {
        Pose2d a = new Pose2d(24, 24, -Math.PI / 2);
        Pose2d b = new Pose2d(24.0000001, 24.0000001, 3 * Math.PI / 2 + 0.000001);

        assertTrue(Geometry.approxNormHeading(a, b));
    }

    @Test
    void testApproxVec() {
        Vector2d a = new Vector2d(482.25592474231, -129.323810145);
        Vector2d b = new Vector2d(482.255924911249, -129.3238109825);

        assertTrue(Geometry.approx(a, b));
    }

    @Test
    void testDistTo() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(3, 4);

        assertEquals(5, Geometry.distTo(a, b));
    }

    @Test
    void testToUserString() {
        Pose2d pose = new Pose2d(86.512, 44.123, Math.PI);
        assertEquals("Pose2d(x=86.5, y=44.1, r=180.0°)", Geometry.toUserString(pose));
    }

    @Test
    void testLerpVec() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(10, 10);

        assertEquals(new Vector2d(5, 5), Geometry.lerp(a, b, 0.5));
    }

    @Test
    void testLerpPose() {
        Pose2d a = new Pose2d(-20, -20, Math.PI / 2);
        Pose2d b = new Pose2d(10, 10, 3 * Math.PI / 2);

        assertEquals(new Pose2d(-12.5, -12.5, 3 * Math.PI / 4), Geometry.lerp(a, b, 0.25));
    }

    @Test
    void testMoveTowardsVec() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(10, 10);

        assertEquals(new Vector2d(5, 5), Geometry.moveTowards(a, b, 5));
    }

    @Test
    void testMoveTowardsPose() {
        Pose2d a = new Pose2d(-20, -20, Math.PI / 2);
        Pose2d b = new Pose2d(10, 10, 3 * Math.PI / 2);

        assertEquals(new Pose2d(-10, -10, Math.PI), Geometry.moveTowards(a, b, 10, Math.PI / 2));
    }

    @Test
    void testSmoothDampVec() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(10, 10);
        Reference<Double> v = new Reference<>(0.0);

        for (int i = 0; i < 5; i++)
            Geometry.smoothDamp(a, b, v, Seconds.of(10), 2, Seconds.of(1));

        assertEquals(1, v.require(), 0.1);
        Vector2d res = Geometry.smoothDamp(a, b, v, Seconds.of(10), 2, Seconds.of(1));
        assertEquals(res.x, 1, 0.1);
        assertEquals(res.y, 1, 0.1);
    }

    @Test
    void testSmoothDampPose() {
        Pose2d a = new Pose2d(0, 0, 0);
        Pose2d b = new Pose2d(10, 10, Math.PI);
        Reference<Double> v = new Reference<>(0.0);

        for (int i = 0; i < 5; i++)
            Geometry.smoothDamp(a, b, v, Seconds.of(10), 10, Seconds.of(1));

        assertEquals(5, v.require(), 0.1);
        Pose2d res = Geometry.smoothDamp(a, b, v, Seconds.of(1), 20, Seconds.of(5));

        assertEquals(10, res.position.x, 0.4);
        assertEquals(10, res.position.y, 0.4);
        assertEquals(20, Mathf.radToDeg(res.heading.log()), 0.8);
    }
}
