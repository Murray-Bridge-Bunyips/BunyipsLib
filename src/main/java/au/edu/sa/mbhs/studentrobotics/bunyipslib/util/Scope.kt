package au.edu.sa.mbhs.studentrobotics.bunyipslib.util

import java.util.function.Consumer
import java.util.function.Function
import java.util.function.Supplier

/**
 * Utility for executing blocks of code within contexts of objects in Java.
 *
 * Optionally possible in Kotlin via [scope functions](https://kotlinlang.org/docs/scope-functions.html).
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
object Scope {
    /**
     * Calls the specified function [block] with [obj] value as its argument and returns its result.
     */
    @JvmStatic
    fun <T, R> let(obj: T, block: Function<T, R>) = block.apply(obj)

    /**
     * Calls the specified function [block] and returns its result.
     */
    @JvmStatic
    fun <T> run(block: Supplier<T>) = block.get()

    /**
     * Calls the specified function [block] and returns no result.
     */
    @JvmStatic
    fun run(block: Runnable) = block.run()

    /**
     * Calls the specified function [block] with [obj] value as its argument and returns [obj].
     */
    @JvmStatic
    fun <T> apply(obj: T, block: Consumer<T>): T {
        block.accept(obj)
        return obj
    }
}
