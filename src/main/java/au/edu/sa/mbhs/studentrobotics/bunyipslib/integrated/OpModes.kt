package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated

import dev.frozenmilk.sinister.sdk.apphooks.SinisterOpModeRegistrar
import dev.frozenmilk.sinister.sdk.opmodes.OpModeScanner
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta

/**
 * Integrated BunyipsLib OpModes.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
object OpModes : SinisterOpModeRegistrar {
    private var suppressOpModes = false

    /**
     * Call this method before start to suppress all BunyipsLib OpModes from appearing in the OpMode list.
     */
    @JvmStatic
    fun suppressOpModes() {
        suppressOpModes = true
    }

    /**
     * Register all the BunyipsLib integrated OpModes.
     *
     * @param registrationHelper used register
     */
    override fun registerOpModes(registrationHelper: OpModeScanner.RegistrationHelper) {
        if (suppressOpModes) return
        registrationHelper.register(
            OpModeMeta.Builder()
                .setName("Hardware Tester")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash") // Using the FtcDashboard group for all of these OpModes to keep the OpMode out of the way
                .build(),
            HardwareTester()
        )
        registrationHelper.register(
            OpModeMeta.Builder()
                .setName("Reset Last Known Positions")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetLastKnowns()
        )
        registrationHelper.register(
            OpModeMeta.Builder()
                .setName("Reset Motor Encoders")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetEncoders()
        )
        registrationHelper.register(
            OpModeMeta.Builder()
                .setName("Reset Robot Controller Lights")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetRobotControllerLights()
        )
    }
}
