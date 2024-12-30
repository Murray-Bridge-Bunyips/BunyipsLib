package au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks

import android.annotation.SuppressLint
import android.content.Context
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BuildConfig
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg
import com.qualcomm.ftccommon.FtcEventLoop
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
import java.lang.reflect.Method

/**
 * BunyipsLib hook manager for the SDK via the `Sinister` utilities, allowing execution of static hooks giving BunyipsLib
 * the ability to hook into the execution cycle of OpModes and access the classpath. The utilities here run in the
 * background regardless of what the user is running, therefore, no interaction is needed with this class.
 *
 * To use the full extent of BunyipsLib utilities at the OpMode level, consult the BunyipsOpMode.
 *
 * @author Lucas Bubner, 2024
 * @since 6.2.0
 */
object BunyipsLib {
    /**
     * Utility to get the OpMode manager for the robot from any context.
     *
     * @return Instance for the OpMode manager
     */
    @SuppressLint("StaticFieldLeak")
    lateinit var opModeManager: OpModeManagerImpl

    private object EventLoopHook : OnCreateEventLoop {
        override fun onCreateEventLoop(context: Context, ftcEventLoop: FtcEventLoop) {
            opModeManager = ftcEventLoop.opModeManager
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
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** INIT OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
        }

        override fun onOpModePreStart(opMode: OpMode) {
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** START OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
        }

        override fun onOpModePostStop(opMode: OpMode) {
            if (opMode is DefaultOpMode) return
            Dbg.log(
                "******************** STOP OPMODE - %: \"%\" ********************",
                opMode.javaClass.simpleName,
                opModeManager.activeOpModeName
            )
            CleanupHookFilter.cleanupMethods.forEach { it.invoke(null) }
        }
    }

    private object CleanupHookFilter : SinisterFilter {
        val cleanupMethods: MutableSet<Method> = mutableSetOf()

        override val targets = FocusedSearch()
            .exclude("com.acmerobotics.roadrunner")
            .exclude("com.acmerobotics.dashboard")
            .exclude("org.team11260.fastload")
            .exclude("org.openftc.easyopencv")
            .exclude("org.opencv")
            .exclude("com.fasterxml.jackson")

        override fun init() {
            cleanupMethods.clear()
        }

        override fun filter(clazz: Class<*>) {
            cleanupMethods.addAll(clazz.declaredMethods.filter {
                it.isStatic() && it.isAnnotationPresent(Cleanup::class.java) && it.parameterCount == 0
            }.onEach { it.isAccessible = true })
        }
    }
}
