package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Unit.Companion.of
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text
import org.firstinspires.ftc.robotcore.external.Telemetry.Item

/**
 * Relay a message in telemetry for a specific amount of time.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
class MessageTask(time: Measure<Time>, private val message: String) : Task() {
    private var item: Item? = null

    init {
        timeout = time
        named("${time to Seconds round 2}s (msg)")
    }

    private fun buildString(): String {
        return Text.format("%/%s: %", elapsedTime to Seconds round 1, timeout to Seconds, message)
    }

    override fun init() {
        item = DualTelemetry.smartAdd(retained = true, format = buildString())
    }

    override fun periodic() {
        item?.setValue(buildString())
    }

    // Ensure the "user condition" is the reason the task ends, so we don't call onInterrupt from timeout
    override fun isTaskFinished() = elapsedTime >= timeout - (1 of Nanoseconds)

    override fun onFinish() {
        if (item != null)
            BunyipsLib.opMode.telemetry.removeItem(item!!)
    }
}