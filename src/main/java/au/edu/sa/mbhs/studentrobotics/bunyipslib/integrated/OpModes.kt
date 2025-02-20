package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import dev.frozenmilk.sinister.apphooks.OpModeRegistrar
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta

/**
 * Integrated BunyipsLib OpModes.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
object OpModes : OpModeRegistrar {
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
     * @param opModeManager used manager
     */
    override fun registerOpModes(opModeManager: AnnotatedOpModeManager) {
        if (suppressOpModes) return
        opModeManager.register(
            OpModeMeta.Builder()
                .setName("Hardware Tester")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash") // Using the FtcDashboard group for all of these OpModes to keep the OpMode out of the way
                .build(),
            HardwareTester::class.java
        )
        opModeManager.register(
            OpModeMeta.Builder()
                .setName("Reset Last Known Positions")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetLastKnowns::class.java
        )
        opModeManager.register(
            OpModeMeta.Builder()
                .setName("Reset Motor Encoders")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetEncoders::class.java
        )
        opModeManager.register(
            OpModeMeta.Builder()
                .setName("Reset Robot Controller Lights")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setGroup("dash")
                .build(),
            ResetRobotControllerLights::class.java
        )
    }
}
