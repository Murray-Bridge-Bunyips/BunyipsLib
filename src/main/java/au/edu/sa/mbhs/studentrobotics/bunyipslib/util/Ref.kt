package au.edu.sa.mbhs.studentrobotics.bunyipslib.util

import dev.frozenmilk.util.cell.Cell
import dev.frozenmilk.util.cell.LazyCell
import dev.frozenmilk.util.cell.RefCell
import java.util.function.Supplier

/**
 * [Cell] construction utilities.
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
object Ref {
    /**
     * Convert [obj] to a [RefCell].
     */
    @JvmStatic
    fun <T> of(obj: T) = RefCell(obj)

    /**
     * Create a [RefCell] that holds null.
     */
    @JvmStatic
    fun empty() = RefCell(null)

    /**
     * Convert this supplier to a [LazyCell].
     */
    @JvmStatic
    fun <T> Supplier<T>.lazy() = LazyCell(this)

    /**
     * Convert this supplier to a [LazyCell].
     */
    fun <T> (() -> T).lazy() = LazyCell(this)

    /**
     * Convert this object to a [RefCell].
     */
    fun <T> T.ref() = RefCell(this)
}