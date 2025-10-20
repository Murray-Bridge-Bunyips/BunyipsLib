package au.edu.sa.mbhs.studentrobotics.bunyipslib.logic

import java.util.function.BooleanSupplier

/**
 * Combines [a] || [b] while preserving [toString].
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
data class Or(val a: BooleanSupplier, val b: BooleanSupplier) : BooleanSupplier {
    override fun getAsBoolean() = a.asBoolean || b.asBoolean
    override fun toString() = "($a || $b)"
}