/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.qualcomm.robotcore.util.RobotLog

/**
 * Provides utility methods for debug logging.
 * @since 1.0.0-pre
 */
object Dbg {
    /**
     * Tag used by Logcat.
     */
    var TAG = "BELLOWER"

    /**
     * Prepended on [error].
     */
    var ERR_PREPEND = "!!"

    /**
     * Prepended on [warn].
     */
    var WRN_PREPEND = "!"

    /**
     * Log an error message.
     * Messages will be prepended with the ERROR_PREPEND string
     * Best used in a scenario where the program cannot continue normally or at required functionality
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun error(stck: StackTraceElement, format: Any, vararg args: Any?) {
        RobotLog.ee(TAG, "$ERR_PREPEND [$stck] ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log an error message.
     * Messages will be prepended with the ERROR_PREPEND string
     * Best used in a scenario where the program cannot continue normally or at required functionality
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun error(obj: Class<*>, format: Any, vararg args: Any?) {
        RobotLog.ee(
            TAG,
            "$ERR_PREPEND [${obj.simpleName}] ${Text.format(format.toString(), *args)}"
        )
    }

    /**
     * Log an error message.
     * Messages will be prepended with the ERROR_PREPEND string
     * Best used in a scenario where the program cannot continue normally or at required functionality
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun error(format: Any, vararg args: Any?) {
        RobotLog.ee(TAG, "$ERR_PREPEND ${Text.format(format.toString(), *args)}")
    }

    /**
     * Report out a stacktrace and cause stacktrace if possible.
     * @param e throwable
     */
    @JvmStatic
    fun sendStacktrace(e: Throwable) {
        fun sendLayer(t: Throwable, tag: String = "") {
            val msg = t.localizedMessage
            RobotLog.ee(TAG, "$tag$t${if (msg == null) "" else " <$msg>"}")
            for (el in t.stackTrace) {
                RobotLog.ee(TAG, "\tat $el")
            }
        }

        sendLayer(e)
        e.cause?.let { sendLayer(it, "caused by: ") }
        e.suppressedExceptions.forEach { sendLayer(it, "suppressed: ") }
    }

    /**
     * Log a warning message.
     * Messages will be prepended with the WRN_PREPEND string
     * Best used in a scenario where the program can continue, but the user should be warned
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun warn(stck: StackTraceElement, format: Any, vararg args: Any?) {
        RobotLog.ww(TAG, "$WRN_PREPEND [$stck] ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log a warning message.
     * Messages will be prepended with the WRN_PREPEND string
     * Best used in a scenario where the program can continue, but the user should be warned
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun warn(obj: Class<*>, format: Any, vararg args: Any?) {
        RobotLog.ww(
            TAG,
            "$WRN_PREPEND [${obj.simpleName}] ${Text.format(format.toString(), *args)}"
        )
    }

    /**
     * Log a warning message.
     * Messages will be prepended with the WRN_PREPEND string
     * Best used in a scenario where the program can continue, but the user should be warned
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun warn(format: Any, vararg args: Any?) {
        RobotLog.ww(TAG, "$WRN_PREPEND ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log an internal debug message.
     * Best used from critical classes to log internal state
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logd(stck: StackTraceElement, format: Any, vararg args: Any?) {
        RobotLog.dd(TAG, "[$stck] ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log an internal debug message.
     * Best used from critical classes to log internal state
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logd(obj: Class<*>, format: Any, vararg args: Any?) {
        RobotLog.dd(
            TAG,
            "[${obj.simpleName}] ${Text.format(format.toString(), *args)}"
        )
    }

    /**
     * Log an internal debug message.
     * Best used from critical classes to log internal state
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logd(format: Any, vararg args: Any?) {
        RobotLog.dd(TAG, Text.format(format.toString(), *args))
    }

    /**
     * Log a user message.
     * Best used to log a message or value to Logcat from user code
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun log(stck: StackTraceElement, format: Any, vararg args: Any?) {
        RobotLog.ii(TAG, "[$stck] ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log a user message.
     * Best used to log a message or value to Logcat from user code
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun log(obj: Class<*>, format: Any, vararg args: Any?) {
        RobotLog.ii(
            TAG,
            "[${obj.simpleName}] ${Text.format(format.toString(), *args)}"
        )
    }

    /**
     * Log a user message.
     * Best used to log a message or value to Logcat from user code
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun log(format: Any, vararg args: Any?) {
        RobotLog.ii(TAG, Text.format(format.toString(), *args))
    }

    /**
     * Log a verbose message.
     * Used from always-messages that can be omitted, such as the firing notifications of methods
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logv(stck: StackTraceElement, format: Any, vararg args: Any?) {
        RobotLog.vv(TAG, "[$stck] ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log a verbose message.
     * Used from always-messages that can be omitted, such as the firing notifications of methods
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logv(obj: Class<*>, format: Any, vararg args: Any?) {
        RobotLog.vv(
            TAG,
            "[${obj.simpleName}] ${Text.format(format.toString(), *args)}"
        )
    }

    /**
     * Log a verbose message.
     * Used from always-messages that can be omitted, such as the firing notifications of methods
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    fun logv(format: Any, vararg args: Any?) {
        RobotLog.vv(TAG, Text.format(format.toString(), *args))
    }

    /**
     * Log a user message temporarily.
     * This method is marked as 'deprecated' to remind you to remove it before committing your code and for it
     * to be picked up as part of static code analysis. It serves the same as a regular [log] call.
     * @param stck StackTraceElement with information about where this log was called (see [Exceptions.getCallingUserCodeFunction])
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    @Deprecated(
        "This method is for temporary debugging only.",
        replaceWith = ReplaceWith("")
    )
    fun logTmp(stck: StackTraceElement, format: Any, vararg args: Any?) {
        log(stck, format, *args)
    }

    /**
     * Log a user message temporarily.
     * This method is marked as 'deprecated' to remind you to remove it before committing your code and for it
     * to be picked up as part of static code analysis. It serves the same as a regular [log] call.
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    @Deprecated(
        "This method is for temporary debugging only.",
        replaceWith = ReplaceWith("")
    )
    fun logTmp(obj: Class<*>, format: Any, vararg args: Any?) {
        log(obj, format, *args)
    }

    /**
     * Log a user message temporarily.
     * This method is marked as 'deprecated' to remind you to remove it before committing your code and for it
     * to be picked up as part of static code analysis. It serves the same as a regular [log] call.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    @JvmStatic
    @Deprecated(
        "This method is for temporary debugging only.",
        replaceWith = ReplaceWith("")
    )
    fun logTmp(format: Any, vararg args: Any?) {
        log(format, *args)
    }

    /**
     * Log a timestamp from inside a BunyipsOpMode.
     * This will call a [log] with the current time since the last timer reset, prefixed with the last user-related
     * function that was called.
     * If not called from a BunyipsOpMode, time information will be unknown.
     */
    @JvmStatic
    fun stamp() {
        log(
            Exceptions.getCallingUserCodeFunction(),
            "Stamped at: T+%s",
            if (BunyipsOpMode.isRunning)
                BunyipsOpMode.instance.timer.elapsedTime() to Seconds
            else
                "?"
        )
    }
}