package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path

import kotlin.math.ceil
import kotlin.math.floor

/**
 * A progression of values of type `Double`.
 *
 * [Source](https://github.com/acmerobotics/road-runner/blob/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/util/DoubleProgression.kt)
 */
data class DoubleProgression(
    /**
     * Start of progression.
     */
    val start: Double,
    /**
     * Step of progression.
     */
    val step: Double,
    /**
     * Size steps of progression.
     */
    val size: Int
) : Iterable<Double> {
    companion object {
        /**
         * Create a new DoubleProgression from the closed interval.
         */
        @JvmStatic
        fun fromClosedInterval(start: Double, endInclusive: Double, count: Int): DoubleProgression {
            val step = when (count) {
                0 -> 0.0
                1 -> 1.0
                else -> (endInclusive - start) / (count - 1)
            }
            return DoubleProgression(start, step, count)
        }
    }

    /**
     * Add two progressions.
     */
    operator fun plus(offset: Double) =
        DoubleProgression(start + offset, step, size)

    /**
     * Subtract two progressions.
     */
    operator fun minus(offset: Double) =
        DoubleProgression(start - offset, step, size)

    /**
     * Multiply this progression by -1.
     */
    operator fun unaryMinus() = DoubleProgression(-start, -step, size)

    /**
     * Whether this progression has nothing in it.
     */
    fun isEmpty() = size == 0

    private fun rawIndex(query: Double) = (query - start) / step

    /**
     * Return a floored index query.
     */
    fun floorIndex(query: Double) = floor(rawIndex(query)).toInt()

    /**
     * Return a ceiled index query.
     */
    fun ceilIndex(query: Double) = ceil(rawIndex(query)).toInt()

    /**
     * Get a position along the progression.
     */
    operator fun get(index: Int) = start + step * index

    /**
     * Whether this progression contains a query.
     */
    operator fun contains(query: Double): Boolean {
        val rawIndex = rawIndex(query)
        return if (rawIndex < 0) {
            false
        } else {
            ceil(rawIndex) < size
        }
    }

    /**
     * Size of the progression.
     */
    fun size() = size

    /**
     * Split this progression into another progression of a custom step.
     */
    fun split(sep: Double): Pair<DoubleProgression, DoubleProgression> {
        val sepIndex = ceilIndex(sep)
        return when {
            sepIndex < 0 -> DoubleProgression(sep, step, 0) to this
            sepIndex >= size -> this to DoubleProgression(sep, step, 0)
            else -> DoubleProgression(start, step, sepIndex) to
                    DoubleProgression(get(sepIndex), step, size - sepIndex)
        }
    }

    /**
     * Iterator implementation for [DoubleProgression].
     */
    @Suppress("IteratorNotThrowingNoSuchElementException")
    inner class IteratorImpl : Iterator<Double> {
        private val iterator: Iterator<Int> = IntRange(0, size - 1).iterator()

        override fun hasNext() = iterator.hasNext()

        override fun next() = get(iterator.next())
    }

    override fun iterator() = IteratorImpl()
}