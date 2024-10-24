package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task

/**
 * Task to wait for a specific amount of time.
 * @since 1.0.0-pre
 */
class WaitTask(time: Measure<Time>, private val showTelemetry: Boolean = true) : Task(time) {
    constructor(time: Measure<Time>) : this(time, true)

    // Special utility constructors for this specific application
    constructor(magnitude: Double, unit: Time) : this(unit.of(magnitude), true)
    constructor(magnitude: Double, unit: Time, showTelemetry: Boolean) : this(unit.of(magnitude), showTelemetry)

    init {
        withName("Wait")
    }

    override fun periodic() {
        if (showTelemetry)
            opMode?.telemetry?.add(
                "Waiting %/% seconds...",
                Mathf.round(deltaTime.to(Seconds), 1),
                timeout.to(Seconds)
            )
    }

    override fun isTaskFinished(): Boolean {
        return false
    }
}