package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * Reset OpMode to clear static memory fields for last known position/alliance.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public final class ResetLastKnowns extends LinearOpMode {
    @Override
    public void runOpMode() {
        Storage.memory().lastKnownPosition = Geometry.zeroPose();
        Storage.memory().lastKnownAlliance = null;
        terminateOpModeNow();
    }
}
