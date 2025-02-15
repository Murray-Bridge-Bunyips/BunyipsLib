package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks.Hook
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingConfiguration
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingPositions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Ref.stringify
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.concurrent.Callable
import java.util.concurrent.Future
import java.util.function.Consumer

/**
 * Async thread to ask for user input from a controller in order to determine a pre-determined
 * set of instructions before a [BunyipsOpMode] starts (`dynamic_init`).
 *
 * You really should only be running one of these threads at a time, preferably using the Threads
 * class to start and manage it to allow for logging and OpMode management. [AutonomousBunyipsOpMode] handles
 * the propagation of a user selection fully automatically and re-exposes it through `onReady`. Note you are
 * able to call this UserSelection manually via [call], but be warned as this will block your thread until completion.
 * During init, this may not matter, so pick whichever method you wish.
 *
 * Keep in mind a thread runs in the background so it is not guaranteed to be ready during any
 * specific phase of your init-cycle. It is recommended to check if this thread is running using
 * `Threads.isRunning(selector)` in your `onInitLoop()` to ensure BOM knows it has to wait for the
 * user to make a selection. Alternatively, you can set an init-task that simply waits for the thread to
 * die or similar (e.g. `Task.task().isFinished(() -> !Threads.isRunning(...))`). You can also alternatively use
 * the init branch of this task to initialise the user selection. `onReady` of [AutonomousBunyipsOpMode] handles this.
 * If you do not implement a waiting action, the OpMode will assume it is ready to run regardless.
 *
 * The result of this thread will be stored in the [Future] getter, which you can access yourself,
 * or you can attach a callback to the `callback` property to be run once the thread is complete.
 * This callback will still be run if the OpMode moves to a running state without a selection. In
 * the event a user does not make a selection, the callback result and `result` property will be
 * null or an array of nulls if chaining is used.
 *
 * ```
 * // In Kotlin using a lambda function, String can be replaced with any type
 * private val selector: UserSelection<String> = UserSelection({ if (it == "POV") initPOVDrive() else initFCDrive() }, "POV", "FIELD-CENTRIC")
 *
 * override fun onInit() {
 *   Threads.start("drive selector", selector)
 * }
 * ```
 *
 * ```
 * // In Java using a callback, String can be replaced with any type
 * private UserSelection<String> selector = new UserSelection<>(this::callback, "POV", "FIELD-CENTRIC");
 *
 * @Override
 * protected void onInit() {
 *   Threads.start("drive selector", selector);
 * }
 *
 * private void callback(@Nullable String res) {
 *   // Do something with res
 * }
 * ```
 *
 * New feature as of BunyipsLib v7.0.0: Passing in multiple [Array] or [Collection] instances, effectively passing
 * in a type [T] of `Array<*>`/`Collection<*>` will activate chaining mode. Chaining mode will prompt the user to select sequentially
 * given the options in each sub-array, allowing you to compose and piece together separate selections. The callback,
 * which will be of `Array<*>` or `Collection<*>` will return the selections made for each section. Null values will
 * fill the array at the point the selection is terminated early, hence the return type is still `T?`.
 * You can choose to disable this behaviour by calling [disableChaining].
 *
 * This class is currently only supported for use in a [BunyipsOpMode] context
 * as it references several of its internal components.
 *
 * @param selections Modes to map to buttons. Will be cast to strings for display and return back in type `T`.
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
class UserSelection<T : Any>(
    /**
     * Runs once the user has made a selection or the thread is interrupted. The result will be the selection made by the user.
     * Can be null if the user did not make a selection.
     */
    private val callback: Consumer<T?>,
    private vararg val selections: T
) : Callable<T> {
    private var disableChaining = false
    private val timer = ElapsedTime()
    private val attentionBorders = arrayOf(
        "<b>---------<font color='red'>!!!</font>--------</b>",
        "<b><font color='red'>---------</font><font color='white'>!!!</font><font color='red'>--------</font></b>"
    )

    companion object {
        private val _lastSelectedButtons = mutableListOf<Controls>()

        /**
         * The buttons that were last selected by the user through an active [UserSelection] instance.
         *
         * Populated in order from prompts that the user was asked for. Auto-cleared at the end of an OpMode.
         *
         * Will be an empty list if no [UserSelection] instances have run, or populated with [Controls.NONE] if
         * a selection ran but no option was selected.
         */
        @JvmStatic
        val lastSelectedButtons: List<Controls>
            get() = _lastSelectedButtons

        @JvmStatic
        @Hook(on = Hook.Target.POST_STOP)
        private fun reset() {
            _lastSelectedButtons.clear()
        }
    }

    /**
     * Disables user selection chaining when passing in sets of [Array] or [Collection] instances.
     *
     * @since 7.0.0
     */
    fun disableChaining() = apply { disableChaining = true }

    /**
     * Maps a set of operation modes to a set of buttons, then blocks until the user inputs an option via `gamepad1`.
     *
     * Currently only supports runtime within a [BunyipsOpMode].
     *
     * @return the result from this UserSelection
     */
    override fun call(): T? {
        if (selections.isEmpty()) {
            Exceptions.runUserMethod { callback.accept(null) }
            return null
        }

        val opMode = BunyipsOpMode.instance

        // Attempt to compose internal arrays or collections into their own layers
        // We lose type clarity, but usually we would be dealing with incompatible types when we have multiple arrays
        val buttonLayers: MutableList<HashMap<Any?, Controls>> = mutableListOf()
        if (disableChaining) {
            buttonLayers.add(Controls.mapArgs(selections))
        } else {
            if (selections.isArrayOf<Array<*>>()) {
                selections.forEach { buttonLayers.add(Controls.mapArgs(it as Array<*>)) }
            } else if (selections.isArrayOf<Collection<*>>()) {
                selections.forEach { buttonLayers.add(Controls.mapArgs((it as Collection<*>).toTypedArray())) }
            } else {
                buttonLayers.add(Controls.mapArgs(selections))
            }
        }

        val results: MutableList<Any?> = mutableListOf()

        buttonLayers.forEachIndexed { index, it ->
            var result: Any? = null
            var selectedButton = Controls.NONE

            var header = "<font color='yellow'><b>(${index + 1}/${buttonLayers.size}) " +
                    "ACTION REQUIRED</b></font>: CHOOSE ON GAMEPAD\n"
            // No point in displaying the number on the DS if there's only one
            if (buttonLayers.size == 1)
                header = header.replaceFirst(Regex("\\(\\d+/\\d+\\)\\s"), "")
            // Dashboard is a backup/secondary so it doesn't really matter if we're overly verbose
            val dashboard = Text.builder("<font color='gray'>(${index + 1}/${buttonLayers.size}) |</font> ")
            val driverStation = Text.builder(header)

            for ((n, button) in it) {
                val name = n.stringify()
                dashboard.append("%: % <font color='gray'>|</font> ", button.name, name)
                driverStation.append(
                    "| %: <b>%</b>\n",
                    button.name,
                    name,
                )
            }

            driverStation.delete(driverStation.length - 1, driverStation.length)

            val topBorder = opMode.telemetry.addDS(attentionBorders[0])
            val mainText = opMode.telemetry.addDS(driverStation)
            val bottomBorder = opMode.telemetry.addDS(attentionBorders[0])
            opMode.telemetry.addDashboard("USR", dashboard)

            var flash = false
            timer.reset()
            while (result == null && opMode.opModeInInit() && !Thread.currentThread().isInterrupted) {
                for ((option, button) in it) {
                    // TODO: debounce
                    if (Controls.isSelected(opMode.gamepad1, button) || Controls.isSelected(opMode.gamepad2, button)) {
                        selectedButton = button
                        result = option
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

            result?.let {
                results.add(it)
                _lastSelectedButtons.add(selectedButton)
            }

            opMode.telemetry.remove(topBorder, mainText, bottomBorder)

            // Early end any nested selections if the OpMode is continuing
            if (result == null) {
                repeat(buttonLayers.size - index) {
                    results.add(null)
                    _lastSelectedButtons.add(Controls.NONE)
                }
                return@forEachIndexed
            }
        }

        val selectionsName = if (results.size > 1)
            "[${results.joinToString { it.stringify() }}]"
        else
            results[0].stringify()

        val buttonsName = if (results.size > 1)
            "[${_lastSelectedButtons.takeLast(buttonLayers.size).joinToString { it.name }}]"
        else
            _lastSelectedButtons.last().name

        if (results.filterNotNull().isNotEmpty()) {
            opMode.telemetry.log("Running: <font color='#caabff'>$buttonsName -> <b>$selectionsName</b></font>")
            opMode.telemetry.addDashboard("USR",
                "$buttonsName -> $selectionsName@T+${opMode.timer.elapsedTime() to Seconds round 1}s")

            // We also auto-parse starting alliance data for Storage, to assist in heuristics
            // Note this only applies for the first found results which is a starting alliance
            results.forEach {
                if (it is StartingPositions) {
                    Storage.memory().lastKnownAlliance = it.toStartingConfiguration().alliance
                    return@forEach
                }
                if (it is StartingConfiguration.Position) {
                    Storage.memory().lastKnownAlliance = it.alliance
                    return@forEach
                }
            }
        } else {
            opMode.telemetry.log("<font color='yellow'>No user selection was made.</font>")
            opMode.telemetry.addDashboard("USR", "No selection")
        }

        //This is code from lucas bubner. He is sad cause hes not important and dosent recieve capital letters. He is lonely except for LACHLAN PAUL  his coding buddy. Now i need to go but always keep this message in mind!!!
        // - Sorayya, hijacker of laptops

        // Finally, we need to cast back into the proper type. We lost type information in the layering stage
        // so we know that if we're dealing with T of some Any, we can cast straight to T as per legacy behaviour.
        // Otherwise, selections that trigger chaining will *have* T to be of Array<*> or Collection<*>,
        // which conveniently matches the results array, and we can simply convert to the proper type.
        // We ignore compiler warnings for this reason.
        @Suppress("UNCHECKED_CAST")
        val userRes: T? = if (results.size > 1) {
            if (selections.isArrayOf<Array<*>>())
                results.toTypedArray() as? T
            else if (selections.isArrayOf<List<*>>())
                results.toCollection(ArrayList()) as? T
            else if (selections.isArrayOf<Set<*>>())
                results.toCollection(LinkedHashSet()) as? T
            else
                throw IllegalArgumentException("Failed to cast back results to type T. This is because the inputted collection object as part of the selection objects is not a List or Set `Collection` type.")
        } else {
            results.first() as? T
        }
        Exceptions.runUserMethod { callback.accept(userRes) }
        return userRes
    }
}
