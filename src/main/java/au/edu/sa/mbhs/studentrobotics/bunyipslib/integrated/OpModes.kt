package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads
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
    private const val RESET_ROBOT_CONTROLLER_LIGHTS_OPMODE = "\$Reset\$RC\$Lights\$"
    private var suppressOpModes = false

    // Have to expose this so HardwareTester can override the Hook filtering (as HardwareTester is integrated)
    @Volatile
    @set:JvmName("__setLightsDirty")
    internal var lightsDirty = false

    /**
     * Call this method before start (such as via an `@OnCreate` hook) to suppress the utility BunyipsLib OpModes from
     * appearing in the OpMode list (Hardware Tester, Reset Last Known Positions, and Reset Motor Encoders).
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
        // Always register the reset lights OpMode
        registrationHelper.register(
            OpModeMeta.Builder()
                .setName(RESET_ROBOT_CONTROLLER_LIGHTS_OPMODE)
                .setFlavor(OpModeMeta.Flavor.SYSTEM)
                .setSystemOpModeBaseDisplayName("Reset Robot Controller Lights")
                .build(),
            ResetRobotControllerLights()
        )
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
            lightsDirty = false
            val name = "schedule reset rc lights opmode"
            if (Threads.isRunning(name))
                return
            Threads.start(name) {
                // We must persist init calls as FtcDashboard handles stops strangely, and the first invocation
                // may not cause the OpMode to run at all. We try to check in with the OpMode to ensure the finisher is run.
                ResetRobotControllerLights.hasInvoked = false
                while (!ResetRobotControllerLights.hasInvoked && !lightsDirty) {
                    BunyipsLib.opModeManager.initOpMode(RESET_ROBOT_CONTROLLER_LIGHTS_OPMODE, true)
                    // We can't poll too fast as OpModes need some time to start/stop
                    Thread.sleep(500)
                }
            }.also { it.ignoreStopAll = true }
        }
    }
}
