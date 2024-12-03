package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.round
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 *
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
class WaitTask(time: Measure<Time>, private val showTelemetry: Boolean = false) : Task() {
    constructor(time: Measure<Time>) : this(time, false)

    // Special utility constructors for this specific application
    constructor(magnitude: Double, unit: Time) : this(unit.of(magnitude), false)
    constructor(magnitude: Double, unit: Time, showTelemetry: Boolean) : this(unit.of(magnitude), showTelemetry)

    init {
        timeout = time
        named("Wait")
    }

    override fun periodic() {
        if (showTelemetry)
            opMode?.telemetry?.add(
                "Waiting %/% seconds...",
                deltaTime to Seconds round 1,
                timeout to Seconds
            )
    }

    override fun isTaskFinished(): Boolean {
        return false
    }
}