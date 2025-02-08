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
     * Attempts to call `toString` on this object while stripping any additional metadata regarding a [Cell], such
     * as stripping `RefCell|Content from wrapped toString()|` to `Content from wrapped toString()`.
     *
     * The standard `toString` call will be executed if this object is not a [Cell].
     *
     * Optionally works on nullable objects which will return "null".
     */
    @JvmStatic
    fun <T> T?.stringify() = if (this is Cell<*>) this.toString().replace(Regex("^.*Cell\\|"), "")
        .replace(Regex("\\|$"), "") else this.toString()

    /**
     * Convert this supplier to a [LazyCell].
     */
    fun <T> (() -> T).lazy() = LazyCell(this)

    /**
     * Convert this object to a [RefCell].
     */
    fun <T> T.ref() = RefCell(this)
}