package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import com.qualcomm.hardware.lynx.LynxModule
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
    private var inhibitNext = false

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
     * Whether to inhibit the next invocation of robot controller lights being reset through the post-stop hook.
     */
    @JvmStatic
    fun inhibitNextLightsReset() {
        inhibitNext = true
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
            if (inhibitNext) {
                Dbg.logv(javaClass, "Robot Controller lights reset operation inhibited by `inhibitNext`.")
                inhibitNext = false
                return
            }
            val rcs = BunyipsLib.opModeManager.hardwareMap.getAll(LynxModule::class.java)
            for (i in rcs.indices) {
                val module = rcs[i]
                Dbg.logv(javaClass, "Resetting Robot Controller (#%) lights from % ...", i + 1, module.pattern)
                module.pattern = LynxModule.blinkerPolicy.getIdlePattern(module)
            }
        }
    }
}
