package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import java.util.Collections
import java.util.function.BooleanSupplier
import kotlin.math.roundToInt

/**
 * Telemetry implementation for BunyipsLib, integrating FtcDashboard and Driver Station calls in one object, while
 * providing additional features useful for debugging and telemetry management.
 * This is used internally by [BunyipsOpMode] to be accessible by the overridden `telemetry` field.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
class DualTelemetry @JvmOverloads constructor(
    private val opMode: OpMode,
    private val movingAverageTimer: MovingAverageTimer? = null,
    /**
     * A tag to prepend to the overhead telemetry status message.
     */
    private val overheadTag: String? = null,
    /**
     * A string to display as the first log message in the telemetry log.
     */
    private val infoString: String? = null
) : Telemetry {
    companion object {
        /**
         * The maximum number of telemetry logs that can be stored in the telemetry log.
         * If the number of logs exceeds this limit, the oldest logs will be removed to make space for new logs to
         * avoid crashing the Driver Station.
         */
        @JvmField
        var TELEMETRY_LOG_LINE_LIMIT = 200
    }

    private lateinit var overheadTelemetry: Item
    private val dashboardItems = Collections.synchronizedSet(mutableSetOf<Pair<ItemType, Reference<String>>>())
    private var userPacket: TelemetryPacket = TelemetryPacket()

    @Volatile
    private var telemetryQueue = 0

    /**
     * A string to display the current 'status' of the OpMode, used for overhead telemetry.
     */
    var opModeStatus = ""

    /**
     * A string that overrules the status message in the overhead telemetry, such as a warning or error message.
     * Set to null to use the default status message.
     */
    var overrideStatus: String? = null

    /**
     * Additional information to display in the overhead telemetry message, as the line under the OpMode name above
     * the timing and controller statistics.
     * By default, this is an empty string.
     */
    var overheadSubtitle = ""

    /**
     * The caption value separator used for automatic splitting of the caption and value for FtcDashboard parsing.
     * This value is also used when calling the aliased but legacy [addData] method (preferred to use [add]).
     *
     * This separator will ensure telemetry that can be added under their own data tags can be done via
     * the dashboard for better logging and graph view operations. (e.g. "Item: Value") will be interpreted as its own
     * item under the Item caption on FtcDashboard. The actual SDK caption-value separator is not affected by this
     * property, and these captions should always be an empty string as captions are not used in DualTelemetry.
     */
    var dashboardCaptionValueAutoSeparator = ": "

    /**
     * The color of the brackets in log messages, useful for distinguishing between different timer phases.
     */
    var logBracketColor = "white"

    /**
     * The time threshold at which the frequency segment will display yellow in the overhead telemetry to alert the user of slow
     * looping times.
     */
    var loopSpeedSlowAlert: Measure<Time> = Milliseconds.of(60.0)

    private enum class ItemType {
        TELEMETRY,
        RETAINED_TELEMETRY,
        LOG
    }

    init {
        clearAll()
        opMode.telemetry.setDisplayFormat(DisplayFormat.HTML)
        opMode.telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
        opMode.telemetry.captionValueSeparator = ""
        opMode.telemetry.log().capacity = TELEMETRY_LOG_LINE_LIMIT
        // Separate log from telemetry on the DS with an empty line
        opMode.telemetry.log().add("")
        if (infoString != null) {
            synchronized(dashboardItems) {
                dashboardItems.add(
                    Pair(
                        ItemType.LOG,
                        Reference.of(infoString)
                    )
                )
            }
            opMode.telemetry.log().add(infoString)
        }
        FtcDashboard.getInstance().withConfigRoot { c ->
            c.putVariable(
                javaClass.simpleName,
                ReflectionConfig.createVariableFromClass(javaClass)
            )
        }
    }

    /**
     * Add a new line to the telemetry object.
     * This is an alias for `add("")`, to signify that a new line is intended.
     */
    fun addNewLine(): HtmlItem {
        return add("")
    }

    /**
     * Add data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * Note that using a separator element (defined by [dashboardCaptionValueAutoSeparator], default ": ") in your formatted string
     * will split this to an item for FtcDashboard automagically, replicating what [addDashboard] would do.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string via [Text.format]
     * @return The telemetry item added to the Driver Station, null if the send failed from overflow
     */
    fun add(format: Any, vararg args: Any?): HtmlItem {
        flushTelemetryQueue()
        var isOverflow = false
        if (telemetryQueue >= 255 && !opMode.telemetry.isAutoClear) {
            // Auto flush will fail as clearing is not permitted
            // We will send telemetry to the debugger instead as a fallback
            Dbg.log("Telemetry overflow: $format")
            isOverflow = true
        }
        return createTelemetryItem(Text.format(format.toString(), *args), false, isOverflow)
    }

    /**
     * Add data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * This is an alias for [add], with downcasting to a normal telemetry item. This will be automagically formatted
     * for FtcDashboard.
     * @param caption Caption before appended separator ([dashboardCaptionValueAutoSeparator])
     * @param format Format string to append after separator ([dashboardCaptionValueAutoSeparator])
     * @param args Objects to format into the format string via [Text.format]
     * @return The telemetry item added to the Driver Station, null if the send failed from overflow
     */
    override fun addData(caption: String, format: String, vararg args: Any?): Item {
        return add(caption + dashboardCaptionValueAutoSeparator + format, *args)
    }

    /**
     * Add data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * This is an alias for [add], with downcasting to a normal telemetry item. This will be automagically formatted
     * for FtcDashboard.
     * @param caption Caption before appended separator ([dashboardCaptionValueAutoSeparator])
     * @param value Value to append after separator ([dashboardCaptionValueAutoSeparator])
     * @return The telemetry item added to the Driver Station, null if the send failed from overflow
     */
    override fun addData(caption: String, value: Any?): Item {
        return add(caption + dashboardCaptionValueAutoSeparator + value)
    }

    /**
     * Add a data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string via [Text.format]
     * @return The telemetry item added to the Driver Station
     */
    fun addRetained(format: Any, vararg args: Any?): HtmlItem {
        flushTelemetryQueue()
        // Retained objects will never be cleared, so we can just add them immediately, as usually
        // retained messages should be important and not be discarded
        return createTelemetryItem(Text.format(format.toString(), *args), true, isOverflow = false)
    }

    /**
     * Add any additional telemetry to the Driver Station telemetry object.
     */
    fun addDS(format: Any, vararg args: Any?): Item {
        return opMode.telemetry.addData("", Text.format(format.toString(), *args))
    }

    /**
     * Get the underlying packet used for user FtcDashboard telemetry in [addDashboard] and [dashboardFieldOverlay].
     */
    fun getDashboardPacket(): TelemetryPacket {
        synchronized(userPacket) {
            return userPacket
        }
    }

    /**
     * Add any additional telemetry to the FtcDashboard telemetry packet.
     */
    fun addDashboard(key: String, value: Any?) {
        synchronized(userPacket) {
            userPacket.put(key, value.toString())
        }
    }

    /**
     * Add any field overlay data to the FtcDashboard telemetry packet.
     */
    fun dashboardFieldOverlay(): Canvas {
        synchronized(userPacket) {
            return userPacket.fieldOverlay()
        }
    }

    /**
     * Remove entries from the Driver Station telemetry object.
     * @param items The items to remove from the DS telemetry object
     */
    fun remove(vararg items: Item): Boolean {
        var ok = true
        for (item in items) {
            // We will be able to remove the item from the DS, but objects on FtcDashboard cannot
            // be removed as we no longer know the index of the object or the contents of the item.
            // This means all RT objects on the dashboard are permanent, and will respect their
            // last updated value. This is not a problem as retained objects are usually important
            // and can serve as a debugging tool.
            var rem = item
            if (item is HtmlItem && item.item != null)
                rem = item.item!!
            val res = opMode.telemetry.removeItem(rem)
            if (!res) {
                Dbg.logd("Could not find telemetry item to remove: $rem")
                ok = false
            }
        }
        return ok
    }

    /**
     * Remove entries from the Driver Station telemetry object.
     * @param items The items to remove from the DS telemetry object
     */
    fun remove(items: List<Item>): Boolean {
        return remove(*items.toTypedArray())
    }

    /**
     * Remove a data item from the telemetry object, which will remove only from the Driver Station.
     * This is an alias for [remove].
     * @param item The item to remove from the telemetry object
     * @see remove
     */
    override fun removeItem(item: Item): Boolean {
        return remove(item)
    }

    private fun logMessage(msg: String) {
        var prepend = ""
        if (movingAverageTimer != null)
            prepend = "<small><font color='$logBracketColor'>[</font>T+${
                Math.round(movingAverageTimer.elapsedTime() to Seconds)
            }s<font color='$logBracketColor'>]</font></small> "
        opMode.telemetry.log().add(prepend + msg)
        synchronized(dashboardItems) {
            dashboardItems.add(Pair(ItemType.LOG, Reference.of(msg)))
        }
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(format: Any, vararg args: Any?) {
        logMessage(Text.format(format.toString(), *args))
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(obj: Class<*>, format: Any, vararg args: Any?) {
        logMessage("<font color='gray'>[${obj.simpleName}]</font> ${Text.format(format.toString(), *args)}")
    }

    /**
     * Log a message into the telemetry log
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(stck: StackTraceElement, format: Any, vararg args: Any?) {
        logMessage("<font color='gray'>[${stck}]</font> ${Text.format(format.toString(), *args)}")
    }

    /**
     * The maximum number of telemetry logs that can be stored in the telemetry log.
     * If the number of logs exceeds this limit, the oldest logs will be removed to make space for new logs to
     * avoid crashing the Driver Station.
     * @param capacity The new capacity for the telemetry log, applied immediately
     */
    fun setLogCapacity(capacity: Int) {
        TELEMETRY_LOG_LINE_LIMIT = capacity
        opMode.telemetry.log().capacity = capacity
    }

    /**
     * Update and push queued telemetry to the Driver Station and FtcDashboard.
     */
    override fun update(): Boolean {
        // Update main DS telemetry
        val retVal = opMode.telemetry.update()

        // Requeue new overhead status message
        val loopTime = movingAverageTimer?.let {
            it.movingAverageLoopTime() to Milliseconds round 2
        } ?: 0.0
        val loopsSec = movingAverageTimer?.let {
            val loopsPerSec = it.loopsPer(Second)
            if (!loopsPerSec.isNaN()) loopsPerSec round 1 else 0.0
        } ?: 0.0
        val elapsedTime = movingAverageTimer?.elapsedTime()?.to(Seconds)?.roundToInt()?.toString() ?: "?"
        val status = if (overrideStatus != null) overrideStatus.toString() else opModeStatus
        val overheadStatus = StringBuilder()
        overheadStatus.append(status).append("\n")
        if (overheadSubtitle.isNotEmpty())
            overheadStatus.append(overheadSubtitle).append("\n")
        overheadStatus.append("<small>T+").append(elapsedTime).append("s | ")
        if (loopTime <= 0.0) {
            if (loopsSec > 0) {
                overheadStatus.append("${loopsSec}Hz")
            } else {
                overheadStatus.append("?ms")
            }
        } else {
            // For LinearOpModes we can suppress any alerts during init as this is the heavy phase of the OpMode
            val noSuppression = opMode !is LinearOpMode || !opMode.opModeInInit()
            if (noSuppression && loopTime >= loopSpeedSlowAlert to Milliseconds) {
                overheadStatus.append("<font color='yellow'>").append(loopTime).append("ms</font>")
            } else {
                overheadStatus.append(loopTime).append("ms")
            }
        }
        overheadStatus.append(" | ")
            .append(Controls.movementString(opMode.gamepad1))
            .append(" ")
            .append(Controls.movementString(opMode.gamepad2))
            .append("</small>\n")
        if (overheadTag != null)
            overheadStatus.insert(0, overheadTag + if (status.isNotEmpty()) " | " else "")
        overheadTelemetry.setValue(overheadStatus.toString())

        // FtcDashboard
        val packet = TelemetryPacket()
        packet.put("STATUS", overheadStatus)

        synchronized(dashboardItems) {
            // Index counters
            var t = 0
            var r = 0
            var l = 0
            // FtcDashboard uses alphabetical order for keys, so we will add a small padding
            // to avoid the dreaded 1, 10, 2 sorting order
            val padding = 3
            dashboardItems.forEach { pair ->
                val (type, ref) = pair
                val currentRef = ref.require()
                val (userTag, value) = let {
                    val parts = currentRef.split(dashboardCaptionValueAutoSeparator, limit = 2)
                    if (parts.size == 2) {
                        Pair(parts[0].trim(), parts[1].trim())
                    } else {
                        Pair(null, parts[0].trim())
                    }
                }
                when (type) {
                    ItemType.TELEMETRY -> packet.put(
                        if (!userTag.isNullOrBlank()) userTag else "DS${String.format("%0${padding}d", t++)}",
                        value
                    )

                    ItemType.RETAINED_TELEMETRY -> packet.put(
                        if (!userTag.isNullOrBlank()) userTag else "RT${String.format("%0${padding}d", r++)}",
                        value
                    )

                    ItemType.LOG -> {
                        if (value == infoString) {
                            // BunyipsLib info, this is an always log and will always
                            // be the first log in the list as it is added at the start
                            // of the init cycle
                            packet.put("INFO", value)
                            return@forEach
                        }
                        // Log items should never be updated so we will use the raw and full value
                        packet.put("LOG${String.format("%0${padding}d", l++)}", currentRef)
                    }
                }
            }
            dashboardItems.removeIf { it.first == ItemType.TELEMETRY }
            // Trim down the FtcDashboard logs to the same amount of logs as the DS
            val logs = dashboardItems.count { it.first == ItemType.LOG }
            if (logs > TELEMETRY_LOG_LINE_LIMIT) {
                val toRemove = dashboardItems.filter { it.first == ItemType.LOG }
                    .take(logs - TELEMETRY_LOG_LINE_LIMIT)
                dashboardItems.removeAll(toRemove.toSet())
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        synchronized(userPacket) {
            FtcDashboard.getInstance().sendTelemetryPacket(userPacket)
            userPacket = TelemetryPacket()
        }

        if (opMode.telemetry.isAutoClear) {
            telemetryQueue = 0
        }

        return retVal
    }

    /**
     * Get the current caption value separator.
     *
     * Note: This value will not represent the actual DS caption value separator, which is always set to an empty
     * string with DualTelemetry, but it will be the separator used with FtcDashboard for automatic telemetry caption/value
     * splitting and legacy addData calls.
     */
    override fun getCaptionValueSeparator(): String {
        return dashboardCaptionValueAutoSeparator
    }

    /**
     * Set the current caption value separator.
     *
     * Note: This value will not represent the actual DS caption value separator, which is always set to an empty
     * string with DualTelemetry, but it will be the separator used with FtcDashboard for automatic telemetry caption/value
     * splitting and legacy addData calls.
     */
    override fun setCaptionValueSeparator(dashboardCaptionValueAutoSeparator: String) {
        this.dashboardCaptionValueAutoSeparator = dashboardCaptionValueAutoSeparator
    }

    /**
     * Clear telemetry on the Driver Station, not including retention
     */
    override fun clear() {
        telemetryQueue = 0
        synchronized(dashboardItems) {
            dashboardItems.removeIf { it.first == ItemType.TELEMETRY }
        }
        opMode.telemetry.clear()
    }

    /**
     * Reset telemetry data, including retention and FtcDashboard
     */
    override fun clearAll() {
        opMode.telemetry.clearAll()
        synchronized(dashboardItems) {
            dashboardItems.removeIf { it.first == ItemType.TELEMETRY }
        }
        FtcDashboard.getInstance().clearTelemetry()
        telemetryQueue = 0
        overheadTelemetry = opMode.telemetry.addData("", "")
            .setRetained(true)
    }

    /**
     * Add an action to perform before Driver Station telemetry is updated.
     */
    override fun addAction(action: Runnable): Any {
        return opMode.telemetry.addAction(action)
    }

    /**
     * Remove an action from the telemetry object.
     */
    override fun removeAction(token: Any): Boolean {
        return opMode.telemetry.removeAction(token)
    }

    /**
     * Speak a message to the Driver Station.
     */
    override fun speak(text: String) {
        opMode.telemetry.speak(text)
    }

    /**
     * Speak a message to the Driver Station with language and country code.
     */
    override fun speak(text: String, languageCode: String, countryCode: String) {
        opMode.telemetry.speak(text, languageCode, countryCode)
    }

    /**
     * Whether the telemetry object is set to auto-clear after each update for the Driver Station.
     * FtcDashboard will always be false.
     */
    override fun isAutoClear(): Boolean {
        return opMode.telemetry.isAutoClear
    }

    /**
     * Set the telemetry object to auto-clear after each update for the Driver Station.
     */
    override fun setAutoClear(autoClear: Boolean) {
        opMode.telemetry.isAutoClear = autoClear
    }

    /**
     * Get the current transmission interval for the Driver Station.
     * FtcDashboard interval can be checked with `FtcDashboard.getInstance().getTelemetryTransmissionInterval()`.
     */
    override fun getMsTransmissionInterval(): Int {
        return opMode.telemetry.msTransmissionInterval
    }

    /**
     * Set the transmission interval in milliseconds for the Driver Station and FtcDashboard.
     */
    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        opMode.telemetry.msTransmissionInterval = msTransmissionInterval
        FtcDashboard.getInstance().telemetryTransmissionInterval = msTransmissionInterval
    }

    /**
     * Set the current display format for the Driver Station.
     * By default this is already set to HTML formatting, and it is recommended to leave this as-is.
     */
    override fun setDisplayFormat(displayFormat: DisplayFormat) {
        opMode.telemetry.setDisplayFormat(displayFormat)
    }

    /**
     * Wraps a telemetry item displayed on the DS/FtcDashboard.
     * Uses HTML to alter the way text is displayed based on wrapper utilities.
     *
     * @author Lachlan Paul, 2024
     */
    class HtmlItem(
        private var value: String,
        retained: Boolean,
        /**
         * Whether this item failed to send to the DS because of telemetry queue overload.
         */
        val isOverflow: Boolean,
        private val dashboardRef: Reference<String>,
        opMode: OpMode
    ) : Item {
        /**
         * The underlying telemetry item added to the DS that this HTML item is wrapping.
         */
        var item: Item? = null
            private set
        private val tags = mutableSetOf<String>()
        private var color: String? = null
        private var bgColor: String? = null
        private var applyOnlyIf: BooleanSupplier? = null

        init {
            if (!isOverflow) {
                // To let dynamic reformatting on an item that already exists, we will use the Func<T> attribute against
                // the HTML string builder, which will ensure changes made after the item has been sent are reflected.
                item = opMode.telemetry.addData("", ::build)
                item?.setRetained(retained)
            }
        }

        private fun build(): String {
            // im david heath, and this is cs50
            if (applyOnlyIf != null && applyOnlyIf?.asBoolean == false) {
                // If the condition evaluates false, we will not apply the HTML formatting
                dashboardRef.set(value)
                return value
            }
            var out = ""
            for (tag in tags)
                out += "<$tag>"
            if (!color.isNullOrEmpty())
                out += "<font color=\"$color\">"
            if (!bgColor.isNullOrEmpty())
                out += "<span style=\"background-color: $bgColor;\">"
            out += value
            if (bgColor != null)
                out += "</span>"
            if (color != null)
                out += "</font>"
            for (tag in tags.reversed())
                out += "</$tag>"
            // Synchronise the FtcDashboard reference
            dashboardRef.set(out)
            return out
        }

        /**
         * Apply the HTML formatting to the string only if if this condition is true.
         */
        fun applyStylesIf(condition: BooleanSupplier): HtmlItem {
            applyOnlyIf = condition
            return this
        }

        /**
         * Wrap the string in a foreground color on the DS.
         */
        fun color(color: String): HtmlItem {
            this.color = color
            build()
            return this
        }

        /**
         * Wrap the string in a background color on the DS.
         */
        fun bgColor(bgColor: String): HtmlItem {
            this.bgColor = bgColor
            build()
            return this
        }

        /**
         * Wrap the string in a tag supplied by the user on the DS. Note that these tags are limited to the HTML tags that
         * are available as part of `Html.fromHtml()`.
         * @param tag The tag to wrap the string in, e.g. "strong" for strong.
         */
        fun wrapWith(tag: String): HtmlItem {
            tags.add(tag)
            build()
            return this
        }

        /**
         * Wrap the string in `<b>` tags, making the string bold on the DS.
         */
        fun bold(): HtmlItem {
            tags.add("b")
            build()
            return this
        }

        /**
         * Wrap the string in `<i>` tags, making the string italic on the DS.
         */
        fun italic(): HtmlItem {
            tags.add("i")
            build()
            return this
        }

        /**
         * Wrap the string in `<big>` tags, making the string large on the DS.
         */
        fun big(): HtmlItem {
            tags.add("big")
            build()
            return this
        }

        /**
         * Wrap the string in `<small>` tags, making the string small on the DS.
         */
        fun small(): HtmlItem {
            tags.add("small")
            build()
            return this
        }

        /**
         * Wrap the string in `<u>` tags, making the string underlined on the DS.
         */
        fun underline(): HtmlItem {
            tags.add("u")
            build()
            return this
        }

        /**
         * Wrap the string in `<s>` tags, making the string strikethrough on the DS.
         */
        fun strikethrough(): HtmlItem {
            tags.add("s")
            build()
            return this
        }

        /**
         * Wrap the string in `<sup>` tags, making the string superscript (top right align) on the DS.
         */
        fun superscript(): HtmlItem {
            tags.add("sup")
            build()
            return this
        }

        /**
         * Wrap the string in `<sub>` tags, making the string subscript (bottom right align) on the DS.
         */
        fun subscript(): HtmlItem {
            tags.add("sub")
            build()
            return this
        }

        /**
         * Wrap the string in `<h1>` tags, making the string a header 1 with a margin on the DS.
         */
        fun h1(): HtmlItem {
            tags.add("h1")
            build()
            return this
        }

        /**
         * Wrap the string in `<h2>` tags, making the string a header 2 with a margin on the DS.
         */
        fun h2(): HtmlItem {
            tags.add("h2")
            build()
            return this
        }

        /**
         * Wrap the string in `<h3>` tags, making the string a header 3 with a margin on the DS.
         */
        fun h3(): HtmlItem {
            tags.add("h3")
            build()
            return this
        }

        /**
         * Wrap the string in `<h4>` tags, making the string a header 4 with a margin on the DS.
         */
        fun h4(): HtmlItem {
            tags.add("h4")
            build()
            return this
        }

        /**
         * Wrap the string in `<h5>` tags, making the string a header 5 with a margin on the DS.
         */
        fun h5(): HtmlItem {
            tags.add("h5")
            build()
            return this
        }

        /**
         * Wrap the string in `<h6>` tags, making the string a header 6 with a margin on the DS.
         */
        fun h6(): HtmlItem {
            tags.add("h6")
            build()
            return this
        }

        /**
         * Returns the caption associated with this item.
         * @return the caption associated with this item.
         * @see setCaption
         * @see addData
         */
        @Deprecated(
            "Captions are not used with DualTelemetry and should always be an empty string.",
            ReplaceWith("\"\""),
            level = DeprecationLevel.ERROR
        )
        override fun getCaption(): String? {
            return item?.caption
        }

        /**
         * Sets the caption associated with this item.
         * @param caption the new caption associated with this item.
         * @return the receiver
         * @see getCaption
         */
        @Deprecated(
            "Captions are not used with DualTelemetry and should always be left as an empty string",
            replaceWith = ReplaceWith(""),
            level = DeprecationLevel.ERROR,
        )
        override fun setCaption(caption: String): Item? {
            return item?.setCaption(caption)
        }

        /**
         * Updates the value of this item to be the result of the indicated string formatting operation.
         * @param format    the format of the data, note this will call a BunyipsLib function to format the string
         * @param args      the arguments associated with the format
         * @return the receiver
         * @see addData
         */
        override fun setValue(format: String, vararg args: Any): Item? {
            value = Text.format(format, *args)
            return item?.setValue(build())
        }

        /**
         * Updates the value of this item to be the result of applying [Object.toString]
         * to the indicated object.
         * @param value the object to which [Object.toString] should be applied
         * @return the receiver
         * @see addData
         */
        override fun setValue(value: Any?): Item? {
            this.value = value.toString()
            return item?.setValue(::build)
        }

        /**
         * Updates the value of this item to be the indicated value producer. This will override any
         * HTML formatting applied to the item as it is also a value producer.
         * @param valueProducer an object that produces values to be rendered.
         * @return the receiver
         * @see addData
         */
        override fun <T : Any> setValue(valueProducer: Func<T>): Item? {
            return item?.setValue(valueProducer)
        }

        /**
         * Updates the value of this item to be the indicated value producer. This will override any
         * HTML formatting applied to the item as it is also a value producer.
         * @param format        this string used to format values produced
         * @param valueProducer an object that produces values to be rendered.
         * @return the receiver
         * @see addData
         */
        override fun <T : Any> setValue(format: String, valueProducer: Func<T>): Item? {
            return item?.setValue(format, valueProducer)
        }

        /**
         * Sets whether the item is to be retained in clear() operation or not.
         * This is initially true for items that whose value is computed with a
         * value producer; otherwise, it is initially false.
         * @param retained if true, then the value will be retained during a clear(). Null will
         * return the setting to its initial value.
         * @return the receiver
         * @see clear
         * @see isRetained
         */
        override fun setRetained(retained: Boolean?): Item? {
            return item?.setRetained(retained)
        }

        /**
         * Returns whether the item is to be retained in a clear() operation.
         * @return whether the item is to be retained in a clear() operation.
         * @see setRetained
         */
        override fun isRetained(): Boolean {
            return item?.isRetained ?: false
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        @Deprecated(
            "This method is not used with DualTelemetry. Split data into separate items or use a single item value with a newline.",
            replaceWith = ReplaceWith("/* split data into seperate items or use a single item with newlines */"),
            level = DeprecationLevel.ERROR
        )
        override fun addData(caption: String, format: String, vararg args: Any): Item? {
            return item?.addData(caption, format, *args)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        @Deprecated(
            "This method is not used with DualTelemetry. Split data into separate items or use a single item value with a newline.",
            replaceWith = ReplaceWith("/* split data into seperate items or use a single item with newlines */"),
            level = DeprecationLevel.ERROR
        )
        override fun addData(caption: String, value: Any): Item? {
            return item?.addData(caption, value)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        @Deprecated(
            "This method is not used with DualTelemetry. Split data into separate items or use a single item value with a newline.",
            replaceWith = ReplaceWith("/* split data into seperate items or use a single item with newlines */"),
            level = DeprecationLevel.ERROR
        )
        override fun <T : Any> addData(caption: String, valueProducer: Func<T>): Item? {
            return item?.addData(caption, valueProducer)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        @Deprecated(
            "This method is not used with DualTelemetry. Split data into separate items or use a single item value with a newline.",
            replaceWith = ReplaceWith("/* split data into seperate items or use a single item with newlines */"),
            level = DeprecationLevel.ERROR
        )
        override fun <T : Any> addData(caption: String, format: String, valueProducer: Func<T>): Item? {
            return item?.addData(caption, format, valueProducer)
        }
    }

    /**
     * Ensure an exception is not thrown due to the telemetry queue being bigger than 255 objects.
     */
    private fun flushTelemetryQueue() {
        telemetryQueue++
        if (telemetryQueue >= 255) {
            // We have to flush out telemetry as the queue is too big
            update()
            if (opMode.telemetry.isAutoClear) {
                // Flush successful
                Dbg.logd("Telemetry queue exceeded 255 messages, auto-pushing to flush...")
            }
        }
    }

    /**
     * Create a new telemetry object and add it to the management queue.
     */
    private fun createTelemetryItem(value: String, retained: Boolean, isOverflow: Boolean): HtmlItem {
        // Use a reference of the string to be added to telemetry. This allows us to pass it into HtmlItem where it
        // may be modified at an arbitrary time, and the changes will be reflected in the telemetry object as part of
        // the telemetry update cycle. This includes all styles to be applied through HtmlItem.
        val ref = Reference.of(value)
        if (value.isNotBlank()) {
            synchronized(dashboardItems) {
                dashboardItems.add(
                    Pair(
                        if (retained) ItemType.RETAINED_TELEMETRY else ItemType.TELEMETRY,
                        ref
                    )
                )
            }
        }
        return HtmlItem(value, retained, isOverflow, ref, opMode)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Function providers are not used via DualTelemetry. Use a polling loop which the current value of the provider, or hook an add() call to telemetry actions.",
        replaceWith = ReplaceWith("add(caption, valueProvider.value())")
    )
    override fun <T : Any> addData(caption: String, valueProducer: Func<T>): Item {
        return add(caption, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Function providers are not used via DualTelemetry. Use a polling loop which the current value of the provider.",
        replaceWith = ReplaceWith("add(caption, valueProvider.value())")
    )
    override fun <T : Any> addData(caption: String, format: String, valueProducer: Func<T>): Item {
        return add(caption + format, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry. This will always return null.",
        replaceWith = ReplaceWith("add(format, args)"),
        level = DeprecationLevel.ERROR
    )
    override fun addLine(): Telemetry.Line? {
        add("")
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry. This will always return null.",
        replaceWith = ReplaceWith("add(format, args)"),
        level = DeprecationLevel.ERROR
    )
    override fun addLine(lineCaption: String): Telemetry.Line? {
        add(lineCaption)
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry. This will always return false.",
        replaceWith = ReplaceWith("add(format, args)"),
        level = DeprecationLevel.ERROR
    )
    override fun removeLine(line: Telemetry.Line): Boolean {
        return false
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry.",
        replaceWith = ReplaceWith("")
    )
    override fun getItemSeparator(): String {
        return opMode.telemetry.itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry.",
        replaceWith = ReplaceWith(""),
        level = DeprecationLevel.ERROR
    )
    override fun setItemSeparator(itemSeparator: String) {
        opMode.telemetry.itemSeparator = itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry log should not be accessed with this method, use log(msg) methods directly as FtcDashboard logging will not work with this method.",
        replaceWith = ReplaceWith("log(...)"),
        level = DeprecationLevel.ERROR
    )
    override fun log(): Telemetry.Log {
        return opMode.telemetry.log()
    }
}