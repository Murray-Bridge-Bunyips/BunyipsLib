package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

/**
 * Integrated BunyipsLib OpModes.
 *
 * @author Lucas Bubner, 2024
 */
public final class OpModes {
    private static boolean suppressOpModes = false;

    private OpModes() {
        throw new AssertionError("No instances");
    }

    /**
     * Call this method before start to suppress all BunyipsLib OpModes from appearing in the OpMode list.
     */
    public static void suppressOpModes() {
        suppressOpModes = true;
    }

    /**
     * Register all the BunyipsLib integrated OpModes.
     *
     * @param manager used manager
     */
    @OpModeRegistrar
    public static void registerOpModes(OpModeManager manager) {
        if (suppressOpModes)
            return;
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Hardware Tester")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        // Using the FtcDashboard group for all of these OpModes to keep the OpMode out of the way
                        .setGroup("dash")
                        .build(),
                new HardwareTester()
        );
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Reset Last Known Positions")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setGroup("dash")
                        .build(),
                new ResetLastKnowns()
        );
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Reset Motor Encoders")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setGroup("dash")
                        .build(),
                new ResetEncoders()
        );
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Reset Robot Controller Lights")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setGroup("dash")
                        .build(),
                new ResetRobotControllerLights()
        );
    }
}
