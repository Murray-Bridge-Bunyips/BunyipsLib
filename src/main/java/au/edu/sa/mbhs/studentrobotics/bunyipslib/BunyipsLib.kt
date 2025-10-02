package au.edu.sa.mbhs.studentrobotics.bunyipslib

import android.content.Context
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib.opMode
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib.opModeManager
import au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated.HardwareTester
import au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated.ResetEncoders
import au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated.ResetLastKnowns
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl.DefaultOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.Version
import dev.frozenmilk.sinister.isStatic
import dev.frozenmilk.sinister.sdk.apphooks.AppHookScanner
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop
import dev.frozenmilk.sinister.targeting.FocusedSearch
import dev.frozenmilk.util.cell.LateInitCell
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.ManualControlOpMode
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import java.lang.reflect.Method
import java.util.function.Consumer

/**
 * BunyipsLib hook manager for the SDK via the `Sinister` utilities, allowing execution of static hooks giving BunyipsLib
 * the ability to hook into the execution cycle of OpModes and access the classpath. The utilities here run in the
 * background regardless of what the user is running, therefore, no interaction is needed with this class.
 *
 * To use the full extent of BunyipsLib utilities at the OpMode level, consult the BunyipsOpMode.
 *
 * @author Lucas Bubner, 2024
 * @since 7.0.0
 */
object BunyipsLib {
    /**
     * Utility to get the event loop for the robot from any context.
     *
     * @return Instance for the FTC Event Loop
     */
    @JvmStatic
    val ftcEventLoop: FtcEventLoop
        get() = _ftcEventLoop.get()

    /**
     * Utility to get the OpMode manager for the robot from any context.
     *
     * @return Instance for the OpMode manager
     */
    @JvmStatic
    val opModeManager: OpModeManagerImpl
        get() = _ftcEventLoop.get().opModeManager

    /**
     * @return The currently active OpMode via [opModeManager] or [BunyipsOpMode] (if available)
     */
    @JvmStatic
    val opMode: OpMode
        get() = if (BunyipsOpMode.isRunning) BunyipsOpMode.instance else opModeManager.activeOpMode

    /**
     * Whether an active *user* OpMode is running. Excludes BunyipsLib-integrated or system OpModes.
     */
    @JvmStatic
    val isUserOpModeRunning: Boolean
        get() = isOpModeUserOpMode(opMode)

    /**
     * Runs [ifRunning] if the [opMode] is an active user OpMode.
     */
    @JvmStatic
    fun ifUserOpModeRunning(ifRunning: Consumer<OpMode>) = if (isUserOpModeRunning) ifRunning.accept(opMode) else Unit

    /**
     * Check if an [opMode] is a user (not System or BunyipsLib-integrated) OpMode.
     */
    @JvmStatic
    fun isOpModeUserOpMode(opMode: OpMode) =
        opMode.let {
            it !is DefaultOpMode
                    && it !is ManualControlOpMode
                    && it !is ResetEncoders
                    && it !is ResetLastKnowns
                    && it !is HardwareTester
                    && it.javaClass.simpleName != "ProcessLoadEvent"
        }

    /**
     * Gets the flavor (Autonomous, TeleOp) of a particular [opMode].
     */
    @JvmStatic
    fun getOpModeFlavor(opMode: OpMode) =
        opMode.javaClass.let {
            if (it.isAnnotationPresent(TeleOp::class.java))
                OpModeMeta.Flavor.TELEOP
            else if (it.isAnnotationPresent(Autonomous::class.java))
                OpModeMeta.Flavor.AUTONOMOUS
            else
                OpModeMeta.Flavor.SYSTEM
        }

    private val _ftcEventLoop = LateInitCell<FtcEventLoop>()

    private object EventLoopHook : OnCreateEventLoop {
        override fun onCreateEventLoop(context: Context, ftcEventLoop: FtcEventLoop) {
            _ftcEventLoop.safeInvoke { it.opModeManager.unregisterListener(OpModeHook) }
            _ftcEventLoop.accept(ftcEventLoop)
            opModeManager.registerListener(OpModeHook)
            Dbg.log(
                "loaded BunyipsLib v% %-% uid:%",
                BuildConfig.SEMVER,
                BuildConfig.GIT_COMMIT,
                BuildConfig.BUILD_TIME,
                BuildConfig.ID
            )
            if (Version.getLibraryVersion() != BuildConfig.SDK_VER) {
                Dbg.error(
                    "SDK version mismatch! (SDK: %, BunyipsLib: %)",
                    Version.getLibraryVersion(),
                    BuildConfig.SDK_VER
                )
                RobotLog.addGlobalWarningMessage("The version of the Robot Controller running on this robot is not the same as the recommended version for BunyipsLib. This may cause incompatibilities. Please ensure you are updated to the SDK version specified in the BunyipsLib documentation.")
            }
        }
    }

    private object OpModeHook : Notifications {
        // Sometimes, our stop handler can double fire when using FtcDashboard. We track this so we don't fire twice.
        private var handledInit = false

        private fun handleOpModeCycle(opMode: OpMode, cycle: Hook.Target) {
            val hooks = mutableSetOf<Pair<Method, Hook>>()
            HookScanner.iterateAppHooks {
                if (it.second.on == cycle)
                    hooks.add(it)
            }
            if (!isOpModeUserOpMode(opMode)) {
                hooks.filter { it.second.ignoreOpModeType }
                    .sortedByDescending { it.second.priority }
                    .forEach {
                        Dbg.logv(
                            javaClass, "invoking %(priority=%, bypassedOpMode='%'): %.%()",
                            cycle.name,
                            it.second.priority,
                            opModeManager.activeOpModeName,
                            it.first.declaringClass.simpleName,
                            it.first.name
                        )
                        Exceptions.runUserMethod { it.first.invoke(null) }
                    }
                return
            }
            if (cycle == Hook.Target.POST_STOP && !handledInit) return
            Dbg.log(
                "******************** % OPMODE - %: \"%\" ********************",
                cycle.name.substringAfter("_"),
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            if (cycle == Hook.Target.PRE_INIT) {
                Threads.start("record metadata") {
                    // Need to wait for the FlightRecorder to be ready
                    val flightLogWriter = FlightRecorder::class.java.getDeclaredField("writer").also { it.isAccessible = true }
                    while (flightLogWriter.get(FlightRecorder) == null && !Thread.currentThread().isInterrupted) {
                        // We don't need a particularly fast poll rate
                        Thread.sleep(500)
                    }
                    FlightRecorder.write("/Metadata/BUILD_TIME", BuildConfig.BUILD_TIME)
                    FlightRecorder.write("/Metadata/GIT_COMMIT", BuildConfig.GIT_COMMIT)
                    FlightRecorder.write("/Metadata/BUILD_UID", BuildConfig.ID)
                    FlightRecorder.write("/Metadata/BUNYIPSLIB_VERSION", BuildConfig.SEMVER)
                    FlightRecorder.write("/Metadata/SDK_VERSION", BuildConfig.SDK_VER)
                    FlightRecorder.write("/Metadata/OPMODE_FLAVOR", getOpModeFlavor(opMode))
                }
            }
            hooks.sortedByDescending { it.second.priority }
                .forEach {
                    Dbg.logv(
                        javaClass, "invoking %(priority=%): %.%() ...",
                        cycle.name,
                        it.second.priority,
                        it.first.declaringClass.simpleName,
                        it.first.name
                    )
                    Exceptions.runUserMethod { it.first.invoke(null) }
                }
            if (cycle == Hook.Target.PRE_INIT) handledInit = true
            if (cycle == Hook.Target.POST_STOP) handledInit = false // Reset for next OpMode or double fire
        }

        override fun onOpModePreInit(opMode: OpMode) = handleOpModeCycle(opMode, Hook.Target.PRE_INIT)
        override fun onOpModePreStart(opMode: OpMode) = handleOpModeCycle(opMode, Hook.Target.PRE_START)
        override fun onOpModePostStop(opMode: OpMode) = handleOpModeCycle(opMode, Hook.Target.POST_STOP)
    }

    private object HookScanner : AppHookScanner<Pair<Method, Hook>>() {
        override val targets = StandardSearch()
        override fun scan(cls: Class<*>, registrationHelper: RegistrationHelper) {
            cls.declaredMethods
                .filter { it.isStatic() && it.isAnnotationPresent(Hook::class.java) && it.parameterCount == 0 }
                .onEach { it.isAccessible = true }
                .map { it to it.getAnnotation(Hook::class.java)!! }
                .forEach { registrationHelper.register(it) }
        }
    }

    /**
     * Scanner target for BunyipsLib + User code.
     */
    class StandardSearch : FocusedSearch() {
        init {
            // Also excludes other libraries relevant to BunyipsLib but won't ever have hook targets
            exclude("com.acmerobotics.roadrunner")
            exclude("com.acmerobotics.dashboard")
            exclude("org.team11260.fastload")
            exclude("org.openftc.easyopencv")
            exclude("org.opencv")
            exclude("com.fasterxml.jackson")
        }
    }
}
