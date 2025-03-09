package au.edu.sa.mbhs.studentrobotics.bunyipslib

import android.content.Context
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl.DefaultOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.Version
import dev.frozenmilk.sinister.isStatic
import dev.frozenmilk.sinister.sdk.apphooks.AppHookScanner
import dev.frozenmilk.sinister.sdk.apphooks.OnCreateEventLoop
import dev.frozenmilk.sinister.targeting.FocusedSearch
import dev.frozenmilk.util.cell.LateInitCell
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.ManualControlOpMode
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
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
     * Utility to get the OpMode manager for the robot from any context.
     *
     * @return Instance for the OpMode manager
     */
    @JvmStatic
    val opModeManager: OpModeManagerImpl
        get() {
            if (!_opModeManager.initialised)
                _opModeManager.accept(OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity))
            return _opModeManager.get()
        }

    /**
     * @return The currently active OpMode via [opModeManager] or [BunyipsOpMode] (if available)
     */
    @JvmStatic
    val opMode: OpMode
        get() = if (BunyipsOpMode.isRunning) BunyipsOpMode.instance else opModeManager.activeOpMode

    /**
     * Whether an active user OpMode is running.
     */
    @JvmStatic
    val isOpModeRunning: Boolean
        get() = opMode !is DefaultOpMode && opMode !is ManualControlOpMode

    /**
     * Runs [ifRunning] if the [opMode] is an active user OpMode.
     */
    @JvmStatic
    fun ifOpModeRunning(ifRunning: Consumer<OpMode>) = if (isOpModeRunning) ifRunning.accept(opMode) else Unit

    private val _opModeManager = LateInitCell<OpModeManagerImpl>()

    private object EventLoopHook : OnCreateEventLoop {
        override fun onCreateEventLoop(context: Context, ftcEventLoop: FtcEventLoop) {
            _opModeManager.safeInvoke { it.unregisterListener(OpModeHook) }
            _opModeManager.accept(ftcEventLoop.opModeManager)
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
        override fun onOpModePreInit(opMode: OpMode) {
            if (opMode is DefaultOpMode || opMode is ManualControlOpMode || opMode.javaClass.simpleName == "ProcessLoadEvent") return
            Dbg.log(
                "******************** INIT OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            val preInit = mutableSetOf<Pair<Method, Hook>>()
            HookScanner.iterateAppHooks {
                if (it.second.on == Hook.Target.PRE_INIT)
                    preInit.add(it)
            }
            preInit.sortedByDescending { it.second.priority }
                .forEach {
                    Dbg.logv(
                        javaClass, "invoking PRE_INIT(priority=%): %.%() ...",
                        it.second.priority,
                        it.first.declaringClass.simpleName,
                        it.first.name
                    )
                    Exceptions.runUserMethod { it.first.invoke(null) }
                }
        }

        override fun onOpModePreStart(opMode: OpMode) {
            if (opMode is DefaultOpMode || opMode is ManualControlOpMode || opMode.javaClass.simpleName == "ProcessLoadEvent") return
            Dbg.log(
                "******************** START OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            val preStart = mutableSetOf<Pair<Method, Hook>>()
            HookScanner.iterateAppHooks {
                if (it.second.on == Hook.Target.PRE_START)
                    preStart.add(it)
            }
            preStart.sortedByDescending { it.second.priority }
                .forEach {
                    Dbg.logv(
                        javaClass, "invoking PRE_START(priority=%): %.%() ...",
                        it.second.priority,
                        it.first.declaringClass.simpleName,
                        it.first.name
                    )
                    Exceptions.runUserMethod { it.first.invoke(null) }
                }
        }

        override fun onOpModePostStop(opMode: OpMode) {
            if (opMode is DefaultOpMode || opMode is ManualControlOpMode || opMode.javaClass.simpleName == "ProcessLoadEvent") return
            Dbg.log(
                "******************** STOP OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            val postStop = mutableSetOf<Pair<Method, Hook>>()
            HookScanner.iterateAppHooks {
                if (it.second.on == Hook.Target.POST_STOP)
                    postStop.add(it)
            }
            postStop.sortedByDescending { it.second.priority }
                .forEach {
                    Dbg.logv(
                        javaClass, "invoking POST_STOP(priority=%): %.%() ...",
                        it.second.priority,
                        it.first.declaringClass.simpleName,
                        it.first.name
                    )
                    Exceptions.runUserMethod { it.first.invoke(null) }
                }
        }
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
