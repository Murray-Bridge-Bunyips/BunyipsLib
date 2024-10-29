package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;

import com.acmerobotics.roadrunner.Pose2d;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingConfiguration;

class StartingConfigurationTest {
    @Test
    void testToFieldPose() {
        Pose2d pose = StartingConfiguration.blueLeft()
                .tile(2.5)
                .backward(Inches.of(4))
                .rotate(Degrees.of(45))
                .build()
                .toFieldPose();
        assertEquals(24, pose.position.x, 1.0e-6);
        assertEquals(64, pose.position.y, 1.0e-6);
        assertEquals(-Math.toRadians(45), pose.heading.toDouble(), 1.0e-6);

        pose = StartingConfiguration.redRight()
                .tile(6)
                .backward(Inches.of(-8))
                .rotate(Degrees.of(-90))
                .build()
                .toFieldPose();
        assertEquals(-60, pose.position.x, 1.0e-6);
        assertEquals(-52, pose.position.y, 1.0e-6);
        assertEquals(0, pose.heading.toDouble(), 1.0e-6);
    }
}
