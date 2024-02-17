package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
import org.murraybridgebunyips.bunyipslib.Text.round
import org.murraybridgebunyips.bunyipslib.roadrunner.util.LynxModuleUtil
import kotlin.math.roundToInt

/**
 * Base class for all OpModes that provides a number of useful methods and utilities for development.
 * Includes a lifecycle that is similar to an iterative lifecycle, but includes logging, error catching,
 * and abstraction to provide phased code execution.
 *
 * @author Lucas Bubner, 2023
 */
abstract class BunyipsOpMode : LinearOpMode() {
    lateinit var movingAverageTimer: MovingAverageTimer
        private set

    private var operationsCompleted = false
    private var operationsPaused = false

    private var opModeStatus = "idle"
    private lateinit var overheadTelemetry: Item
    private var telemetryQueue = 0
    private val telemetryObjects = mutableListOf<Item>()
    private val logItems = mutableListOf<String>()
    private val extraDashboardItems = mutableListOf<Pair<String, String>>()

    companion object {
        /**
         * The instance of the current BunyipsOpMode. This is set automatically by the OpMode lifecycle.
         * This can be used instead of dependency injection to access the current OpMode, as it is a singleton.
         *
         * BunyipsComponent and Task internally use this to grant access to the current OpMode through
         * the `opMode` property. You must ensure all Tasks and BunyipsComponents are instantiated during runtime,
         * (such as during onInit()), otherwise this property will be null.
         */
        @JvmStatic
        lateinit var instance: BunyipsOpMode
            private set
    }

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract fun onInit()

    /**
     * Run code in a loop AFTER onInit has completed, until
     * start is pressed on the Driver Station or true is returned to this method.
     * If not implemented, the OpMode will continue on as normal and wait for start.
     */
    protected open fun onInitLoop(): Boolean {
        return true
    }

    /**
     * Allow code to execute once after all initialisation has finished.
     * Note: this method is always called even if initialisation is cut short by the driver station.
     */
    protected open fun onInitDone() {
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike onInitDone, this will only execute once play is hit and not when initialisation is done.
     */
    protected open fun onStart() {
    }

    /**
     * Code to run when the START button is pressed on the Driver Station.
     * This method will be called on each hardware cycle.
     */
    protected abstract fun activeLoop()

    /**
     * Perform one time clean-up operations after the OpMode finishes.
     * This method is not exception protected!
     */
    protected open fun onStop() {
    }

    /**
     * Main method overridden from LinearOpMode that handles the OpMode lifecycle.
     * @throws InterruptedException
     */
    @Throws(InterruptedException::class)
    final override fun runOpMode() {
        // BunyipsOpMode
        instance = this
        try {
            Dbg.log("=============== BunyipsLib BunyipsOpMode ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME} uid:${BuildConfig.ID} ===============")
            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
            // Separate log from telemetry on the DS with an empty line
            telemetry.log().add("")
            telemetry.log().add("bunyipslib ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME}")
            logItems.add("bunyipslib ${BuildConfig.GIT_COMMIT}-${BuildConfig.BUILD_TIME}")

            opModeStatus = "setup"
            Dbg.logd("BunyipsOpMode: setting up...")
            telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
            telemetry.captionValueSeparator = ""
            // Uncap the telemetry log limit to ensure we capture everything
            telemetry.log().capacity = 999999
            // Ring-buffer timing utility
            movingAverageTimer = MovingAverageTimer()
            // Assign overhead telemetry as it has been configured now
            overheadTelemetry =
                telemetry.addData("", "BOM: idle | T+0s | 0.000ms | (?) (?)\n")
                    .setRetained(true)
            pushTelemetry()

            opModeStatus = "static_init"
            Dbg.logd("BunyipsOpMode: firing onInit()...")
            // Store telemetry objects raised by onInit() by turning off auto-clear
            setTelemetryAutoClear(false)
            pushTelemetry()
            if (!gamepad1.atRest() || !gamepad2.atRest()) {
                log("WARNING: a gamepad was not zeroed during init. please ensure controllers zero out correctly.")
            }
            // Run user-defined setup
            try {
                onInit()
            } catch (e: Exception) {
                // All InterruptedExceptions are handled by the FTC SDK and are raised in ErrorUtil
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            opModeStatus = "dynamic_init"
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: starting onInitLoop()...")
            // Run user-defined dynamic initialisation
            while (opModeInInit()) {
                try {
                    // Run until onInitLoop returns true or the OpMode is continued
                    if (onInitLoop()) break
                } catch (e: Exception) {
                    ErrorUtil.handleCatchAllException(e, ::log)
                }
                movingAverageTimer.update()
                pushTelemetry()
            }

            opModeStatus = "finish_init"
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: firing onInitDone()...")
            // Run user-defined final initialisation
            try {
                onInitDone()
            } catch (e: Exception) {
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            // Ready to go.
            opModeStatus = "ready"
            movingAverageTimer.update()
            Dbg.logd("BunyipsOpMode: init cycle completed in ${movingAverageTimer.elapsedTime() / 1000.0} secs")
            // DS only telemetry
            telemetry.addData("", "BunyipsOpMode: INIT COMPLETE -- PLAY WHEN READY.")
            Dbg.logd("BunyipsOpMode: ready.")
            movingAverageTimer.reset()

            // Wait for start
            do {
                pushTelemetry()
            } while (!isStarted)

            // Play button has been pressed
            opModeStatus = "starting"
            setTelemetryAutoClear(true)
            clearTelemetry()
            pushTelemetry()
            movingAverageTimer.reset()
            Dbg.logd("BunyipsOpMode: firing onStart()...")
            try {
                // Run user-defined start operations
                onStart()
            } catch (e: Exception) {
                ErrorUtil.handleCatchAllException(e, ::log)
            }

            opModeStatus = "running"
            Dbg.logd("BunyipsOpMode: starting activeLoop()...")
            while (opModeIsActive() && !operationsCompleted) {
                if (operationsPaused) {
                    // If the OpMode is paused, skip the loop and wait for the next hardware cycle
                    opModeStatus = "halted"
                    movingAverageTimer.update()
                    pushTelemetry()
                    continue
                }
                try {
                    // Run user-defined active loop
                    activeLoop()
                } catch (e: Exception) {
                    ErrorUtil.handleCatchAllException(e, ::log)
                }
                // Update telemetry and timers
                movingAverageTimer.update()
                pushTelemetry()
            }

            opModeStatus = "finished"
            // overheadTelemetry will no longer update, will remain frozen on last value
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: all tasks finished.")
            // Wait for user to hit stop or for the OpMode to be terminated
            while (opModeIsActive()) {
                // no-op
            }
        } catch (t: Throwable) {
            Dbg.error("BunyipsOpMode: unhandled throwable! <${t.message}>")
            Dbg.sendStacktrace(t)
            // As this error occurred somewhere not inside user code, we should not swallow it.
            // There is also a chance this error is an instance of Error, in which case we should
            // exit immediately as something has gone very wrong
            throw t
        } finally {
            Dbg.logd("BunyipsOpMode: opmode stop requested. cleaning up...")
            // Ensure all user threads have been told to stop
            Threads.stopAll()
            onStop()
            // Telemetry may be not in a nice state, so we will call our stateful functions
            // such as thread stops and cleanup in onStop() first before updating the status
            opModeStatus = "terminating"
            Dbg.logd("BunyipsOpMode: active cycle completed in ${movingAverageTimer.elapsedTime() / 1000.0} secs")
            pushTelemetry()
            Dbg.logd("BunyipsOpMode: exiting...")
        }
    }

    /**
     * Update and push queued telemetry to the Driver Station and FtcDashboard.
     */
    fun pushTelemetry() {
        // Update main DS telemetry
        telemetry.update()

        // Requeue new overhead status message
        val loopTime = round(movingAverageTimer.movingAverage(), 2)
        val loopsSec = if (!movingAverageTimer.loopsSec().isNaN())
            round(movingAverageTimer.loopsSec(), 1)
        else 0.0

        val overheadStatus =
            "$opModeStatus | T+${(movingAverageTimer.elapsedTime() / 1000).roundToInt()}s | ${
                if (loopTime <= 0.0) {
                    if (loopsSec > 0) "$loopsSec l/s" else "0.00ms"
                } else {
                    "${loopTime}ms"
                }
            } | ${Controller.movementString(gamepad1)} ${Controller.movementString(gamepad2)}\n"

        overheadTelemetry.setValue("BOM: $overheadStatus")

        // FtcDashboard
        val packet = TelemetryPacket()
        packet.put("BOM", overheadStatus + "\n")
        for ((key, value) in extraDashboardItems) {
            packet.put(key, value)
        }
        // Copy with toList() to avoid ConcurrentModificationExceptions
        telemetryObjects.toList().forEachIndexed { index, item ->
            packet.put("DS$index", item.caption)
        }
        logItems.toList().forEachIndexed { index, item ->
            if (index == 0) {
                // BunyipsLib info, this is an always log
                packet.put("INFO", item)
                return@forEachIndexed
            }
            packet.put("LOG$index", item)
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        if (telemetry.isAutoClear) {
            telemetryQueue = 0
            clearTelemetryObjects()
        }
    }

    /**
     * Add any additional telemetry to the FtcDashboard telemetry packet.
     */
    fun addDashboardTelemetry(key: String, value: String) {
        if (extraDashboardItems.contains(Pair(key, value))) {
            extraDashboardItems.remove(Pair(key, value))
        }
        extraDashboardItems.add(Pair(key, value))
    }

    /**
     * Ensure an exception is not thrown due to the telemetry object being bigger than 250 objects.
     */
    private fun flushTelemetryQueue() {
        telemetryQueue++
        if (telemetryQueue >= 250) {
            // We have to flush out telemetry as the queue is too big
            pushTelemetry()
            if (telemetry.isAutoClear) {
                // Flush successful
                Dbg.logd("Telemetry queue exceeded 250 messages, auto-pushing to flush...")
            }
        }
    }

    /**
     * Create a new telemetry object and add it to the management queue.
     */
    private fun createTelemetryItem(value: String, retained: Boolean): Item {
        // Must store everything in the caption as the value cannot be accessed,
        // and all formatting is already done via BunyipsLib
        val item = telemetry.addData(value, "")
            .setRetained(retained)
        telemetryObjects.add(item)
        return item
    }

    /**
     * Clear all telemetry objects from the management queue just like a telemetry.clear() call.
     */
    private fun clearTelemetryObjects() {
        val tmp = telemetryObjects.toTypedArray()
        for (item in tmp) {
            if (item.isRetained)
                continue
            telemetryObjects.remove(item)
        }
    }

    /**
     * Add data to the telemetry object
     * @param value A string to add to telemetry
     * @return The telemetry item added to the Driver Station
     */
    fun addTelemetry(value: String): Item? {
        flushTelemetryQueue()
        if (telemetryQueue >= 250 && !telemetry.isAutoClear) {
            // Auto flush will fail as clearing is not permitted
            // We will send telemetry to the debugger instead as a fallback
            Dbg.log("Telemetry overflow: $value")
            return null
        }
        return createTelemetryItem(value, false)
    }

    /**
     * Add data to the telemetry object
     * @param value An object to add to telemetry
     * @return The telemetry item added to the Driver Station
     */
    fun addTelemetry(value: Any): Item? {
        return addTelemetry(value.toString())
    }

    /**
     * Add data to the telemetry object using a custom format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     * @return The telemetry item added to the Driver Station
     */
    fun addTelemetry(fstring: String, vararg objs: Any): Item? {
        return addTelemetry(formatString(fstring, objs.asList()))
    }

    /**
     * Add retained non-auto-clearing data to the telemetry object
     * @param value A string to add to telemetry
     * @return The telemetry item added to the Driver Station
     */
    fun addRetainedTelemetry(value: String): Item {
        flushTelemetryQueue()
        // Retained telemetry will never be auto-cleared so we'll just send the object now
        return createTelemetryItem(value, true)
    }

    /**
     * Add a data to the telemetry object using a custom format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     * @return The telemetry item added to the Driver Station
     */
    fun addRetainedTelemetry(fstring: String, vararg objs: Any): Item {
        return addRetainedTelemetry(formatString(fstring, objs.asList()))
    }

    /**
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetainedTelemetry(vararg items: Item) {
        for (item in items) {
            val res = telemetry.removeItem(item)
            if (!res) {
                Dbg.logd("Could not find telemetry item to remove: $item")
                return
            }
        }
    }

    fun removeRetainedTelemetry(items: List<Item>) {
        removeRetainedTelemetry(*items.toTypedArray())
    }

    /**
     * Log a message to the telemetry log
     * @param message The message to log
     */
    fun log(message: String) {
        telemetry.log().add(message)
        logItems.add(message)
    }

    /**
     * Log a message to the telemetry log using a format string
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     */
    fun log(fstring: String, vararg objs: Any) {
        log(formatString(fstring, objs.asList()))
    }

    /**
     * Log a message to the telemetry log
     * @param obj Class where this log was called (name will be prepended to message in lowercase)
     * @param message The message to log
     */
    fun log(obj: Class<*>, message: String) {
        val msg = "[${obj.simpleName.lowercase()}] $message"
        telemetry.log().add(msg)
        logItems.add(msg)
    }

    /**
     * Log a message into the telemetry log
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param message The message to log
     */
    fun log(stck: StackTraceElement, message: String) {
        val msg = "[${stck}] $message"
        telemetry.log().add(msg)
        logItems.add(msg)
    }

    /**
     * Log a message to the telemetry log using a format string
     * @param obj Class where this log was called (name will be prepended to message in lowercase)
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     */
    fun log(obj: Class<*>, fstring: String, vararg objs: Any) {
        log(obj, formatString(fstring, objs.asList()))
    }

    /**
     * Log a message into the telemetry log using a format string
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param fstring A format string to add to telemetry
     * @param objs The objects to format into the string
     */
    fun log(stck: StackTraceElement, fstring: String, vararg objs: Any) {
        log(stck, formatString(fstring, objs.asList()))
    }

    /**
     * Reset telemetry data, including retention
     */
    fun resetTelemetry() {
        telemetryQueue = 0
        telemetry.clearAll()
        telemetryObjects.clear()
        FtcDashboard.getInstance().clearTelemetry()
        // Must reassign the overhead telemetry item
        overheadTelemetry =
            telemetry.addData("", "BOM: unknown | T+?s | ?ms | (?) (?)")
                .setRetained(true)
    }

    /**
     * Clear telemetry on screen, not including retention
     */
    fun clearTelemetry() {
        telemetryQueue = 0
        clearTelemetryObjects()
        telemetry.clear()
    }

    /**
     * Set telemetry auto clear status
     */
    fun setTelemetryAutoClear(autoClear: Boolean) {
        telemetry.isAutoClear = autoClear
    }

    /**
     * Get auto-clear status of telemetry
     */
    fun getTelemetryAutoClear(): Boolean {
        return telemetry.isAutoClear
    }

    /**
     * Call to manually finish the OpMode.
     * This is a dangerous method, as the OpMode will no longer be able to run any main thread code.
     * This method should be called when the OpMode is finished and no longer needs to run, and will
     * will put the OpMode in a state where it will not run any more code (including timers & telemetry).
     */
    fun finish() {
        if (operationsCompleted) {
            return
        }
        operationsCompleted = true
        Dbg.logd("BunyipsOpMode: activeLoop() terminated by finish().")
        createTelemetryItem(
            "BunyipsOpMode: Robot is stopped. All operations completed.",
            true
        )
        pushTelemetry()
    }

    /**
     * Call to temporarily halt all activeLoop-related updates from running.
     * Note this will pause the entire activeLoop, but continue to update timers and telemetry. These events
     * must be handled manually if needed, which include any conditional calls to resume().
     */
    fun halt() {
        if (operationsPaused) {
            return
        }
        operationsPaused = true
        Dbg.logd("BunyipsOpMode: activeLoop() halted.")
    }

    /**
     * Call to resume the activeLoop after a halt() call.
     */
    fun resume() {
        if (!operationsPaused) {
            return
        }
        operationsPaused = false
        opModeStatus = "running"
        Dbg.logd("BunyipsOpMode: activeLoop() resumed.")
    }

    /**
     * Dangerous method: call to shut down the OpMode as soon as possible.
     * This will run any BunyipsOpMode cleanup code.
     */
    fun exit() {
        Dbg.logd("BunyipsOpMode: exiting opmode...")
        finish()
        requestOpModeStop()
    }

    /**
     * Dangerous method: call to IMMEDIATELY terminate the OpMode.
     * This will not run any cleanup code, and should only be used in emergencies.
     */
    fun emergencyStop() {
        Dbg.logd("BunyipsOpMode: emergency stop requested.")
        terminateOpModeNow()
    }
}
