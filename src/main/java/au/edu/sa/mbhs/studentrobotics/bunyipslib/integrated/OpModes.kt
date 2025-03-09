package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook
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
    private const val RESET_ROBOT_CONTROLLER_LIGHTS_OPMDOE = "\$Reset\$RC\$Lights\$"
    private var lightsDirty = false
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
                .setName(RESET_ROBOT_CONTROLLER_LIGHTS_OPMDOE)
                .setFlavor(OpModeMeta.Flavor.SYSTEM)
                .setSystemOpModeBaseDisplayName("Reset Robot Controller Lights")
                .build(),
            ResetRobotControllerLights()
        )
    }

    @JvmStatic
    @Hook(on = Hook.Target.PRE_INIT, priority = Int.MAX_VALUE)
    private fun markRobotControllerLightsAsDirty() {
        lightsDirty = true
    }

    @JvmStatic
    @Hook(on = Hook.Target.POST_STOP, priority = -Int.MAX_VALUE, ignoreOpModeType = true)
    private fun tryResetRobotControllerLights() {
        if (lightsDirty) {
            BunyipsLib.opModeManager.initOpMode(RESET_ROBOT_CONTROLLER_LIGHTS_OPMDOE, true)
            lightsDirty = false
        }
    }
}
