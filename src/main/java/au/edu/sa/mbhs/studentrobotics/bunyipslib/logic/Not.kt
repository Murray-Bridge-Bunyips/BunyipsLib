package au.edu.sa.mbhs.studentrobotics.bunyipslib.logic

import java.util.function.BooleanSupplier

/**
 * Inverts [a] to ![a] while preserving [toString].
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
data class Not(val a: BooleanSupplier) : BooleanSupplier {
    override fun getAsBoolean() = !a.asBoolean
    override fun toString() = "!($a)"
}