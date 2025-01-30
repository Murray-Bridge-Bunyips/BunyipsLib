package au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks

import android.content.Context
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BuildConfig
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl.DefaultOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.Version
import dev.frozenmilk.sinister.SinisterFilter
import dev.frozenmilk.sinister.apphooks.OnCreateEventLoop
import dev.frozenmilk.sinister.isStatic
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
            // Manual reload all FtcDashboard entries, this makes it so Restart Robot will also refresh dashboard
            for (clazz in DashFilter.loaded) {
                FtcDashboard.getInstance()?.withConfigRoot {
                    val name = clazz.getAnnotation(Config::class.java)!!.value.ifEmpty { clazz.simpleName }
                    it.putVariable(name, ReflectionConfig.createVariableFromClass(clazz))
                }
            }
        }
    }

    private object OpModeHook : Notifications {
        override fun onOpModePreInit(opMode: OpMode) {
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** INIT OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            HookFilter.preInit.forEach { it.invoke(null) }
        }

        override fun onOpModePreStart(opMode: OpMode) {
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** START OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            HookFilter.preStart.forEach { it.invoke(null) }
        }

        override fun onOpModePostStop(opMode: OpMode) {
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** STOP OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            HookFilter.postStop.forEach { it.invoke(null) }
        }
    }

    private object HookFilter : SinisterFilter {
        val preInit = LinkedHashSet<Method>()
        val preStart = LinkedHashSet<Method>()
        val postStop = LinkedHashSet<Method>()

        override val targets = StdSearch()

        override fun init() {
            preInit.clear()
            preStart.clear()
            postStop.clear()
        }

        override fun filter(clazz: Class<*>) {
            val allHooks = clazz.declaredMethods.filter {
                it.isStatic() && it.isAnnotationPresent(Hook::class.java) && it.parameterCount == 0
            }.onEach { it.isAccessible = true }

            allHooks.map { it to it.getAnnotation(Hook::class.java)!! }
                .sortedByDescending { it.second.priority }
                .forEach {
                    when (it.second.on) {
                        Hook.Target.PRE_INIT -> preInit.add(it.first)
                        Hook.Target.PRE_START -> preStart.add(it.first)
                        Hook.Target.POST_STOP -> postStop.add(it.first)
                    }
                }
        }
    }

    private object DashFilter : SinisterFilter {
        val loaded = mutableSetOf<Class<*>>()

        override val targets = StdSearch()

        override fun init() {
            loaded.clear()
        }

        override fun filter(clazz: Class<*>) {
            if (!clazz.isAnnotationPresent(Config::class.java) || clazz.isAnnotationPresent(Disabled::class.java)) return
            loaded.add(clazz)
        }
    }

    /**
     * [SinisterFilter] target for BunyipsLib + User code.
     */
    class StdSearch : FocusedSearch() {
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
