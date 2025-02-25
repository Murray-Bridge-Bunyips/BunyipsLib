package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks.Hook
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl.ForceStopException
import java.io.PrintWriter
import java.io.StringWriter
import java.util.function.Consumer

/**
 * OpMode "userspace" util to prevent user-code unhandled exceptions from crashing the app.
 *
 * This will log the exception and stacktrace to the Driver Station, allowing OpModes to continue running.
 * In the past, exceptions used to cause an EMERGENCY STOP condition, but has changed to a more modern popup window,
 * however, this class is still useful as it will not terminate the OpMode and will allow code to continue
 * while providing full logging in both Logcat and the Driver Station.
 *
 * @since 1.0.0-pre
 */
object Exceptions {
    /**
     * Exceptions thrown since the last reset of this set.
     */
    @JvmField
    val THROWN_EXCEPTIONS = mutableSetOf<Throwable>()

    /**
     * Maximum number of characters to display in the stacktrace on the Driver Station.
     */
    @JvmField
    var MAX_DS_STACKTRACE_CHARS = 250

    /**
     * Handle an exception, logging it to the Driver Station and Logcat.
     *
     * @param e The exception to handle.
     * @param stderr The function to print logging to the Driver Station or other stderr outputs that support HTML parsing.
     * @see runUserMethod for single method executions of exception-prone user code
     */
    @JvmStatic
    fun handle(e: Throwable, stderr: Consumer<String>?) {
        var out = stderr
        Dbg.error("Exception caught! Stacktrace:")
        Dbg.sendStacktrace(e)
        if (THROWN_EXCEPTIONS.stream().anyMatch { it.toString() == e.toString() } || !THROWN_EXCEPTIONS.add(e)) {
            // Don't log out the same exception twice
            out = null
        }
        val sw = StringWriter()
        e.printStackTrace(PrintWriter(sw))
        var stack = sw.toString()
        if (stack.length > MAX_DS_STACKTRACE_CHARS) {
            stack = stack.substring(0, MAX_DS_STACKTRACE_CHARS - 4)
            stack += " ..."
        }
        out?.accept("<font color='red'><b>exception caught!</b></font> -> <small>$stack</small>")
        // We will bother writing out the cause but not bother about suppressed exceptions, since this is user-facing
        // Logcat via sendStacktrace will contain all info if required, so we don't lose info
        val ec = e.cause
        if (ec != null) {
            var cause = ec.toString()
            if (cause.length > MAX_DS_STACKTRACE_CHARS) {
                cause = cause.substring(0, MAX_DS_STACKTRACE_CHARS - 4)
                cause += " ..."
            }
            out?.accept("caused by: $cause")
        }
        if (e is InterruptedException || e is ForceStopException) {
            Dbg.error("Interrupt exception called, raising to superclass...")
            // FTC SDK must handle this
            throw e
        }
        if (e is EmergencyStop) {
            Dbg.error("Emergency stop exception called, raising to superclass...")
            // This is a special exception where we shouldn't continue running the OpMode
            // We will let the FTC SDK handle it in terminating the OpMode and displaying the popup
            throw e
        }
    }

    /**
     * Handle and run a single user-controlled method by wrapping the execution with an `Exceptions.handle()`.
     *
     * This ensures integrity in methods that are written by a user to be more graceful in execution,
     * ensuring code execution is not stopped on a non-critical exception.
     */
    @JvmStatic
    fun runUserMethod(method: Runnable) {
        try {
            method.run()
        } catch (e: Exception) {
            handle(e) { DualTelemetry.smartLog(it) }
        }
    }

    /**
     * Get the calling user code function of the current context by looking at the stacktrace until it leaves BunyipsLib.
     */
    @JvmStatic
    fun getCallingUserCodeFunction(): StackTraceElement {
        val stackTrace = Thread.currentThread().stackTrace
        // Keep going down the stack trace until we leave the BunyipsLib package
        for (stackTraceElement in stackTrace) {
            // dalvik.system.VMStack.getThreadStackTrace(Native Method) is not useful, which shows up in the stacktrace
            if (stackTraceElement.toString().contains("stacktrace", true)) continue
            // If porting, ensure the string below is set to the package name of BunyipsLib
            if (!stackTraceElement.className.startsWith(BuildConfig.LIBRARY_PACKAGE_NAME)) {
                return stackTraceElement
            }
        }
        // If we can't find the calling function, then we can't return a stack trace element
        Dbg.warn("Could not find calling function in getCallingUserCodeFunction()!")
        return StackTraceElement("Unknown", "userMethod", "User Code", -1)
    }

    @Hook(on = Hook.Target.POST_STOP)
    @JvmStatic
    private fun reset() {
        THROWN_EXCEPTIONS.clear()
    }
}