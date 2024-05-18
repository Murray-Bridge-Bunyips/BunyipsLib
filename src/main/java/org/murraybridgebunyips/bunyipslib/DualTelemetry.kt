package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat
import org.firstinspires.ftc.robotcore.external.Telemetry.Item
import org.murraybridgebunyips.bunyipslib.Text.formatString
import org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds
import org.murraybridgebunyips.bunyipslib.external.units.Units.Second
import org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds
import java.util.Collections
import kotlin.math.roundToInt

/**
 * Telemetry implementation for BunyipsLib, integrating FtcDashboard and Driver Station calls in one object, while
 * providing additional features useful for debugging and telemetry management.
 * This is used internally by [BunyipsOpMode] to be accessible by the overridden `telemetry` field.
 *
 * @author Lucas Bubner, 2024
 */
@Config
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
    infoString: String? = null
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
    private val dashboardItems = Collections.synchronizedSet(mutableSetOf<Pair<ItemType, String>>())

    @Volatile
    private var telemetryQueue = 0

    @Volatile
    private var packet: TelemetryPacket = TelemetryPacket()

    /**
     * A string to display the current 'status' of the OpMode, used for overhead telemetry.
     */
    var opModeStatus = "idle"

    private enum class ItemType {
        TELEMETRY,
        RETAINED_TELEMETRY,
        LOG
    }

    init {
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
                        infoString
                    )
                )
            }
            opMode.telemetry.log().add(infoString)
        }
    }

    /**
     * Add data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
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
        return createTelemetryItem(formatString(format.toString(), *args), false, isOverflow)
    }

    /**
     * Add a data to the telemetry object for the Driver Station and FtcDashboard, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     * @return The telemetry item added to the Driver Station
     */
    fun addRetained(format: Any, vararg args: Any?): HtmlItem {
        flushTelemetryQueue()
        // Retained objects will never be cleared, so we can just add them immediately, as usually
        // retained messages should be important and not be discarded
        return createTelemetryItem(formatString(format.toString(), *args), true, isOverflow = false)
    }

    /**
     * Add any additional telemetry to the Driver Station telemetry object.
     */
    fun addDS(format: Any, vararg args: Any?): Item {
        return opMode.telemetry.addData("", formatString(format.toString(), *args))
    }

    /**
     * Add any additional telemetry to the FtcDashboard telemetry packet.
     */
    fun addDashboard(key: String, value: Any?) {
        packet.put(key, value.toString())
    }

    /**
     * Add any field overlay data to the FtcDashboard telemetry packet.
     */
    fun dashboardFieldOverlay(): Canvas {
        return packet.fieldOverlay()
    }

    /**
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetained(vararg items: Item): Boolean {
        var ok = true
        for (item in items) {
            // We will be able to remove the item from the DS, but objects on FtcDashboard cannot
            // be removed as we no longer know the index of the object or the contents of the item.
            // This means all RT objects on the dashboard are permanent, and will respect their
            // last updated value. This is not a problem as retained objects are usually important
            // and can serve as a debugging tool.
            val res = opMode.telemetry.removeItem(item)
            if (!res) {
                Dbg.logd("Could not find telemetry item to remove: $item")
                ok = false
            }
        }
        return ok
    }

    /**
     * Remove retained entries from the telemetry object.
     * @param items The items to remove from the telemetry object
     */
    fun removeRetained(items: List<Item>) {
        removeRetained(*items.toTypedArray())
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(format: Any, vararg args: Any?) {
        val fstring = formatString(format.toString(), *args)
        opMode.telemetry.log().add(fstring)
        synchronized(dashboardItems) {
            dashboardItems.add(Pair(ItemType.LOG, fstring))
        }
    }

    /**
     * Log a message to the telemetry log, with integrated formatting.
     * @param obj Class where this log was called (name will be prepended to message)
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(obj: Class<*>, format: Any, vararg args: Any?) {
        val msg = "[${obj.simpleName}] ${formatString(format.toString(), *args)}"
        opMode.telemetry.log().add(msg)
        synchronized(dashboardItems) {
            dashboardItems.add(Pair(ItemType.LOG, msg))
        }
    }

    /**
     * Log a message into the telemetry log
     * @param stck StackTraceElement with information about where this log was called (see Text.getCallingUserCodeFunction())
     * @param format An object string to add to telemetry
     * @param args The objects to format into the object format string
     */
    fun log(stck: StackTraceElement, format: Any, vararg args: Any?) {
        val msg = "[${stck}] ${formatString(format.toString(), *args)}"
        opMode.telemetry.log().add(msg)
        synchronized(dashboardItems) {
            dashboardItems.add(Pair(ItemType.LOG, msg))
        }
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
        val loopTime =
            if (movingAverageTimer != null) Text.round(
                movingAverageTimer.movingAverageLoopTime().inUnit(Milliseconds),
                2
            ) else 0.0
        val loopsSec = if (movingAverageTimer != null)
            if (!movingAverageTimer.loopsPer(Second).isNaN())
                Text.round(movingAverageTimer.loopsPer(Second), 1)
            else 0.0
        else 0.0
        val elapsedTime = movingAverageTimer?.elapsedTime()?.inUnit(Seconds)?.roundToInt() ?: "?"

        val overheadStatus =
            "$opModeStatus | T+${elapsedTime}s | ${
                if (loopTime <= 0.0) {
                    if (loopsSec > 0) "$loopsSec l/s" else "?ms"
                } else {
                    "${loopTime}ms"
                }
            } | ${Controls.movementString(opMode.gamepad1)} ${Controls.movementString(opMode.gamepad2)}\n"

        overheadTelemetry.setValue("${if (overheadTag != null) "$overheadTag: " else ""}$overheadStatus")

        // FtcDashboard
        packet.let {
            synchronized(it) {
                it.put(overheadTag ?: "status", overheadStatus + "\n")

                dashboardItems.forEachIndexed { index, pair ->
                    val (type, value) = pair
                    when (type) {
                        ItemType.TELEMETRY -> it.put("DS$index", value)
                        ItemType.RETAINED_TELEMETRY -> it.put("RT$index", value)
                        ItemType.LOG -> {
                            if (index == 0) {
                                // BunyipsLib info, this is an always log and will always
                                // be the first log in the list as it is added at the start
                                // of the init cycle
                                it.put("INFO", value)
                                return@forEachIndexed
                            }
                            it.put("LOG$index", value)
                        }
                    }
                }

                FtcDashboard.getInstance().sendTelemetryPacket(it)

                // Invalidate this packet
                packet = TelemetryPacket()
            }
        }

        if (opMode.telemetry.isAutoClear) {
            telemetryQueue = 0
            clearTelemetryObjects()
        }

        return retVal
    }

    /**
     * Clear telemetry on the Driver Station, not including retention
     */
    override fun clear() {
        telemetryQueue = 0
        clearTelemetryObjects()
        opMode.telemetry.clear()
    }

    /**
     * Reset telemetry data, including retention and FtcDashboard
     */
    override fun clearAll() {
        opMode.telemetry.clearAll()
        synchronized(dashboardItems) {
            dashboardItems.clear()
        }
        FtcDashboard.getInstance().clearTelemetry()
        telemetryQueue = 0
        overheadTelemetry = opMode.telemetry.addData(
            "",
            "${if (overheadTag != null) "$overheadTag: " else ""}unknown | T+?s | ?ms | (?) (?))\n"
        )
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
     * Get the current caption value separator for the Driver Station. This should always be an empty string as
     * BunyipsTelemtry does not use captions.
     */
    override fun getCaptionValueSeparator(): String {
        return opMode.telemetry.captionValueSeparator
    }

    /**
     * Get the current display format for the Driver Station.
     */
    override fun setDisplayFormat(displayFormat: DisplayFormat) {
        opMode.telemetry.setDisplayFormat(displayFormat)
    }

    /**
     * Setup and reset the telemetry object for the start of an OpMode.
     */
    fun setup() {
        clearAll()
        opModeStatus = "setup"
        telemetryQueue = 0
        synchronized(dashboardItems) {
            dashboardItems.clear()
        }
        packet = TelemetryPacket()
    }

    /**
     * Uses HTML to alter the way text is displayed on the Driver Station.
     * Wraps a telemetry item displayed on the DS.
     *
     * @author Lachlan Paul, 2024
     */
    class HtmlItem(
        private val value: String,
        retained: Boolean,
        /**
         * Whether this item failed to send to the DS because of telemetry queue overload.
         */
        val isOverflow: Boolean,
        opMode: OpMode
    ): Item {
        private var item: Item? = null

        private var size: Int? = null
        private var colour: String? = null
        private var isBold: Boolean = false
        private var isItalic: Boolean = false
        private var isUnderlined: Boolean = false

        init {
            if (!isOverflow) {
                // To let dynamic reformatting on an item that already exists, we will use the Func<T> attribute against
                // the HTML string builder, which will ensure changes made after the item has been sent are reflected.
                item = opMode.telemetry.addData("", ::buildFormattedString)
                item?.setRetained(retained)
            }
        }

        /**
         * Change the size of the string.
         * @param px The size of the string in pixels
         * @return The HtmlItem backing the DS item
         */
        fun setSize(px: Int): HtmlItem {
            size = px
            return this
        }

        /**
         * Change the colour of the string.
         * @param colour The colour of the string in HTML format, in any CSS colour format (hex, rgb(), name, etc.)
         * @return The HtmlItem backing the DS item
         */
        fun setColour(colour: String): HtmlItem {
            this.colour = colour
            return this
        }

        /**
         * Change whether the string is bold.
         * @param bold Whether the string is bold
         * @return The HtmlItem backing the DS item
         */
        fun setBold(bold: Boolean): HtmlItem {
            isBold = bold
            return this
        }

        /**
         * Makes the string bold.
         * @return The HtmlItem backing the DS item
         */
        fun bold(): HtmlItem {
            isBold = true
            return this
        }

        /**
         * Change whether the string is italic.
         * @param italic Whether the string is italic
         * @return The HtmlItem backing the DS item
         */
        fun setItalic(italic: Boolean): HtmlItem {
            isItalic = italic
            return this
        }

        /**
         * Makes the string italic.
         * @return The HtmlItem backing the DS item
         */
        fun italic(): HtmlItem {
            isItalic = true
            return this
        }

        /**
         * Change whether the string is underlined.
         * @param underlined Whether the string is underlined
         * @return The HtmlItem backing the DS item
         */
        fun setUnderlined(underlined: Boolean): HtmlItem {
            isUnderlined = underlined
            return this
        }

        /**
         * Makes the string underlined.
         * @return The HtmlItem backing the DS item
         */
        fun underlined(): HtmlItem {
            isUnderlined = true
            return this
        }

        /**
         * Builds the HTML string in relation to the current styling.
         * This method does **not need to be called** in order to send your string to the DS telemetry.
         * @return the HTML span string with the added styling from previous builder instruction
         */
        private fun buildFormattedString(): String {
            // im david heath, and this is cs50
            var styleString = "<span style=\""

            if (size != null)
                styleString += "font-size:${size}px;"
            if (colour != null)
                styleString += "color:${colour};"
            if (isBold)
                styleString += "font-weight:bold;"
            if (isItalic)
                styleString += "font-style:italic;"
            if (isUnderlined)
                styleString += "text-decoration:underline;"

            return "$styleString\">${value}</span>"
        }

        /**
         * Returns the caption associated with this item.
         * @return the caption associated with this item.
         * @see setCaption
         * @see addData
         */
        override fun getCaption(): String? {
            return item?.caption
        }

        /**
         * Sets the caption associated with this item.
         * @param caption the new caption associated with this item.
         * @return the receiver
         * @see getCaption
         */
        override fun setCaption(caption: String): Item? {
            return item?.setCaption(caption)
        }

        /**
         * Updates the value of this item to be the result of the indicated string formatting operation.
         * @param format    the format of the data
         * @param args      the arguments associated with the format
         * @return the receiver
         * @see addData
         */
        override fun setValue(format: String, vararg args: Any): Item? {
            return item?.setValue(format, args)
        }

        /**
         * Updates the value of this item to be the result of applying [Object.toString]
         * to the indicated object.
         * @param value the object to which [Object.toString] should be applied
         * @return the receiver
         * @see addData
         */
        override fun setValue(value: Any): Item? {
            return item?.setValue(value)
        }

        /**
         * Updates the value of this item to be the indicated value producer.
         * @param valueProducer an object that produces values to be rendered.
         * @return the receiver
         * @see addData
         */
        override fun <T : Any> setValue(valueProducer: Func<T>): Item? {
            return item?.setValue(valueProducer)
        }

        /**
         * Updates the value of this item to be the indicated value producer.
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
        override fun addData(caption: String, format: String, vararg args: Any): Item? {
            return item?.addData(caption, format, args)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        override fun addData(caption: String, value: Any): Item? {
            return item?.addData(caption, value)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
        override fun <T : Any> addData(caption: String, valueProducer: Func<T>): Item? {
            return item?.addData(caption, valueProducer)
        }

        /**
         * Adds a new data item in the associated [Telemetry] immediately following the receiver.
         * @see addData
         */
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
        if (value.isNotBlank()) {
            synchronized(dashboardItems) {
                dashboardItems.add(
                    Pair(
                        if (retained) ItemType.RETAINED_TELEMETRY else ItemType.TELEMETRY,
                        value
                    )
                )
            }
        }
        return HtmlItem(value, false, isOverflow, opMode)
    }

    /**
     * Clear all telemetry objects from the management queue just like a telemetry.clear() call.
     */
    private fun clearTelemetryObjects() {
        synchronized(dashboardItems) {
            val tmp = dashboardItems.toTypedArray()
            for (item in tmp) {
                if (item.first == ItemType.RETAINED_TELEMETRY)
                    continue
                dashboardItems.remove(item)
            }
        }
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry", replaceWith = ReplaceWith("add(caption + format, args)"))
    override fun addData(caption: String, format: String, vararg args: Any): Item {
        return add(caption + format, args)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry", replaceWith = ReplaceWith("add(caption + value)"))
    override fun addData(caption: String, value: Any): Item {
        return add(caption + value)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Captions and function providers are not used with DualTelemetry",
        replaceWith = ReplaceWith("add(caption) // Use polling loop and fstring for provider")
    )
    override fun <T : Any> addData(caption: String, valueProducer: Func<T>): Item {
        return add(caption, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "Captions and function providers are not used with DualTelemetry",
        replaceWith = ReplaceWith("add(caption + format) // Use polling loop and fstring for provider")
    )
    override fun <T : Any> addData(caption: String, format: String, valueProducer: Func<T>): Item {
        return add(caption + format, valueProducer.value())
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "removeItem has been migrated to removeRetained with usages in retained telemetry (will still work for non-retained)",
        replaceWith = ReplaceWith("removeRetained(item)")
    )
    override fun removeItem(item: Item): Boolean {
        return removeRetained(item)
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun addLine(): Telemetry.Line? {
        add("")
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun addLine(lineCaption: String): Telemetry.Line? {
        add(lineCaption)
        return null
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun removeLine(line: Telemetry.Line): Boolean {
        return false
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun getItemSeparator(): String {
        return opMode.telemetry.itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry line system is not used with DualTelemetry",
        replaceWith = ReplaceWith("add(format, args)")
    )
    override fun setItemSeparator(itemSeparator: String) {
        opMode.telemetry.itemSeparator = itemSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated("Captions are not used with DualTelemetry and should always be left as an empty string")
    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        opMode.telemetry.captionValueSeparator = captionValueSeparator
    }

    @Suppress("KDocMissingDocumentation")
    @Deprecated(
        "The telemetry log should not be accessed with this method, use log(msg) methods directly as FtcDashboard logging will not work with this method.",
        replaceWith = ReplaceWith("log(...)")
    )
    override fun log(): Telemetry.Log {
        return opMode.telemetry.log()
    }
}