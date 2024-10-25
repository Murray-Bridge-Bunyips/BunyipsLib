package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.TrapezoidProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.ProfiledPIDController;

class ProfiledPIDControllerTest {
    @Test
    void testStartFromNonZeroPosition() {
        ProfiledPIDController controller = new ProfiledPIDController(1.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(1.0, 1.0));

        controller.reset(20);

        assertEquals(0, controller.calculate(20, 20), 0.05);
    }
}