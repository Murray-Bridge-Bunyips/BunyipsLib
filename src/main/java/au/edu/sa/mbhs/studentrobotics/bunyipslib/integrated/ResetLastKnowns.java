package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * Reset OpMode to clear static memory fields for last known position/alliance.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public final class ResetLastKnowns {
    private static boolean suppress = false;

    private ResetLastKnowns() {
    }

    /**
     * Call before start to suppress the OpMode from appearing in the OpMode list.
     */
    public static void suppressOpMode() {
        suppress = true;
    }

    /**
     * Register the OpMode.
     *
     * @param manager The OpModeManager to register the OpMode with.
     */
    @OpModeRegistrar
    public static void registerOpMode(OpModeManager manager) {
        if (suppress)
            return;
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Reset Last Known Positions")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        // Using the FtcDashboard group to keep the OpMode out of the way
                        .setGroup("dash")
                        .build(),
                new LinearOpMode() {
                    @Override
                    public void runOpMode() {
                        Storage.memory().lastKnownPosition = Geometry.zeroPose();
                        Storage.memory().lastKnownAlliance = null;
                        terminateOpModeNow();
                    }
                }
        );
    }
}
