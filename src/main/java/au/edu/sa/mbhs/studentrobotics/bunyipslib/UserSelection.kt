package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingConfiguration
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingPositions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.Consumer

/**
 * Async thread to ask for user input from a controller in order to determine a pre-determined
 * set of instructions before a [BunyipsOpMode] starts (`dynamic_init`).
 *
 * You really should only be running one of these threads at a time, preferably using the Threads
 * class to start and manage it to allow for logging and OpMode management.
 *
 * Keep in mind this thread runs in the background so it is not guaranteed to be ready during any
 * specific phase of your init-cycle. It is recommended to check if this thread is running using
 * `Threads.isRunning(selector)` in your `onInitLoop()` to ensure BOM knows it has to wait for the
 * user to make a selection. Alternatively, you can set an init-task that simply waits for the thread to
 * die or similar (e.g. `Task.task().isFinished(() -> !Threads.isRunning(...))`).
 * If you do not do this, the OpMode will assume it is ready to run regardless.
 *
 * The result of this thread will be stored in the `result` property, which you can access yourself,
 * or you can attach a callback to the `callback` property to be run once the thread is complete.
 * This callback will still be run if the OpMode moves to a running state without a selection. In
 * the event a user does not make a selection, the callback result and `result` property will be
 * null.
 *
 * ```
 * // In Kotlin using a lambda function, String can be replaced with any type
 * private val selector: UserSelection<String> = UserSelection({ if (it == "POV") initPOVDrive() else initFCDrive() }, "POV", "FIELD-CENTRIC")
 *
 * override fun onInit() {
 *   Threads.start(selector)
 * }
 * ```
 *
 * ```
 * // In Java using a callback, String can be replaced with any type
 * private UserSelection<String> selector = new UserSelection<>(this::callback, "POV", "FIELD-CENTRIC");
 *
 * @Override
 * protected void onInit() {
 *   Threads.start(selector);
 * }
 *
 * private void callback(@Nullable String res) {
 *   // Do something with res
 * }
 * ```
 *
 * `res` will be null if the user did not make a selection.
 *
 * Updated to use dynamic button mapping and generics 04/08/23.
 * Updated to be async and removed time restriction 07/09/23.
 *
 * @param opModes Modes to map to buttons. Will be cast to strings for display and return back in type `T`.
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
class UserSelection<T : Any>(
    /**
     * Runs once the user has made a selection or the thread is interrupted. The result will be the selection made by the user.
     * Can be null if the user did not make a selection.
     */
    private val callback: Consumer<T?>,
    private vararg val opModes: T
) : BunyipsComponent(), Runnable {
    private val timer = ElapsedTime()

    /**
     * The result of the user selection. Will be null if the user did not make a selection.
     * Passed into the callback.
     */
    @Volatile
    var result: T? = null
        private set

    /**
     * The button that was selected by the user.
     */
    var selectedButton: Controls = Controls.NONE
        private set

    /**
     * Maps a set of operation modes to a set of buttons.
     * @return A HashMap of operation modes to buttons.
     */
    override fun run() {
        if (opModes.isEmpty()) {
            Exceptions.runUserMethod(opMode) { callback.accept(null) }
            return
        }

        val buttons: HashMap<T, Controls> = Controls.mapArgs(opModes)

        val attentionBorders = arrayOf(
            "<b>---------<font color='red'>!!!</font>--------</b>",
            "<b><font color='red'>---------</font><font color='white'>!!!</font><font color='red'>--------</font></b>"
        )
        val driverStation =
            Text.builder("<font color='yellow'><b>ACTION REQUIRED</b></font>: INIT OPMODE WITH GAMEPAD 1\n")
        val dashboard = Text.builder("<font color='gray'>|</font> ")

        for ((name, button) in buttons) {
            dashboard.append("%: % <font color='gray'>|</font> ", button.name, name)
            driverStation.append(
                "| %: <b>%</b>\n",
                button.name,
                name,
            )
        }

        driverStation.delete(driverStation.length - 1, driverStation.length)

        val topBorder = require(opMode).telemetry.addDS(attentionBorders[0])
        val mainText = require(opMode).telemetry.addDS(driverStation)
        val bottomBorder = require(opMode).telemetry.addDS(attentionBorders[0])
        require(opMode).telemetry.addDashboard("USR", dashboard)

        var flash = false
        while (result == null && require(opMode).opModeInInit() && !Thread.currentThread().isInterrupted) {
            for ((str, button) in buttons) {
                if (Controls.isSelected(require(opMode).gamepad1, button)) {
                    selectedButton = button
                    result = str
                    break
                }
            }
            if (timer.seconds() > 0.5) {
                flash = !flash
                timer.reset()
            }
            if (flash) {
                topBorder.setValue(attentionBorders[1])
                bottomBorder.setValue(attentionBorders[1])
            } else {
                topBorder.setValue(attentionBorders[0])
                bottomBorder.setValue(attentionBorders[0])
            }
            // Updates will be handled by the main telemetry loop
        }

        val opModeName = result.toString()

        if (result == null) {
            require(opMode).telemetry.log("<font color='yellow'>No user OpMode selection was made.</font>")
        } else {
            require(opMode).telemetry.log("Running OpMode: <font color='#caabff'>${selectedButton.name} -> <b>$opModeName</b></font>")
            if (result is StartingPositions) {
                Storage.memory().lastKnownAlliance = (result as StartingPositions).toStartingConfiguration().alliance
            } else if (result is StartingConfiguration.Position) {
                Storage.memory().lastKnownAlliance = (result as StartingConfiguration.Position).alliance
            }
        }

        require(opMode).telemetry.addDashboard(
            "USR",
            if (result == null) "No selection" else "${selectedButton.name} -> $opModeName@T+${
                require(opMode).timer.elapsedTime() to Seconds round 1
            }s"
        )

        //This is code from lucas bubner. He is sad cause hes not important and dosent recieve capital letters. He is lonely except for LACHLAN PAUL  his coding buddy. Now i need to go but always keep this message in mind!!!
        // - Sorayya, hijacker of laptops

        // Clean up telemetry
        require(opMode).telemetry.remove(topBorder, mainText, bottomBorder)

        Exceptions.runUserMethod(opMode) { callback.accept(result) }
    }
}
