// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units

import java.util.Locale
import kotlin.math.abs

/**
 * A measure holds the magnitude and unit of some dimension, such as distance, time, or speed. Two
 * measures with the same *unit* and *magnitude* are effectively equivalent objects.
 *
 * @param U the unit type of the measure
 * @since 1.0.0-pre
 */
interface Measure<U : Unit<U>> : Comparable<Measure<U>> {
    /**
     * Gets the unitless magnitude of this measure.
     *
     * @return the magnitude in terms of this unit.
     */
    fun magnitude(): Double

    /**
     * Gets the magnitude of this measure in terms of the base unit. If the unit is the base unit for
     * its system of measure, then the value will be equivalent to [magnitude].
     *
     * @return the magnitude in terms of the base unit
     */
    fun baseUnitMagnitude(): Double

    /**
     * Gets the units of this measure.
     *
     * @return the unit
     */
    fun unit(): U

    /**
     * Converts this measure to a measure with a different unit of the same type, e.g. minutes to
     * seconds. Converting to the same unit is equivalent to calling [magnitude].
     *
     * For Kotlin users, calling this method can be done with the notation \`in\`
     * (see [here](https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin)),
     * or by calling the alias `to`.
     *
     * ```
     * Meters.of(12).in(Feet) // 39.3701
     * Seconds.of(15).in(Minutes) // 0.25
     * ```
     *
     * @param unit the unit to convert this measure to
     * @return the value of this measure in the given unit
     */
    infix fun `in`(unit: Unit<U>): Double {
        return if (unit() == unit) {
            magnitude()
        } else {
            unit.fromBaseUnits(baseUnitMagnitude())
        }
    }

    /**
     * Converts this measure to a measure with a different unit of the same type, e.g. minutes to
     * seconds. Converting to the same unit is equivalent to calling [magnitude].
     *
     * ```
     * Meters.of(12).in(Feet) // 39.3701
     * Seconds.of(15).in(Minutes) // 0.25
     * ```
     *
     * @param unit the unit to convert this measure to
     * @return the value of this measure in the given unit
     */
    infix fun to(unit: Unit<U>): Double {
        return `in`(unit)
    }

    /**
     * Multiplies this measurement by some constant multiplier and returns the result. The magnitude
     * of the result will be the *base* magnitude multiplied by the scalar value. If the measure
     * uses a unit with a non-linear relation to its base unit (such as Fahrenheit for temperature),
     * then the result will only be a multiple *in terms of the base unit*.
     *
     * @param multiplier the constant to multiply by
     * @return the resulting measure
     */
    operator fun times(multiplier: Double): Measure<U> {
        return ImmutableMeasure.ofBaseUnits(baseUnitMagnitude() * multiplier, unit())
    }

    /**
     * Generates a new measure that is equal to this measure multiplied by another. Some dimensional
     * analysis is performed to reduce the units down somewhat; for example, multiplying a `Measure<Time>` by a `Measure<Velocity<Distance>>` will return just a `Measure<Distance>` instead of the naive `Measure<Mult<Time, Velocity<Distance>>`. This is
     * not guaranteed to perform perfect dimensional analysis.
     *
     * @param U2 the type of the other measure to multiply by
     * @param other the unit to multiply by
     * @return the multiplicative unit
     */
    operator fun <U2 : Unit<U2>> times(other: Measure<U2>): Measure<*> {
        if (other.unit() is Dimensionless) {
            // scalar multiplication
            return times(other.baseUnitMagnitude())
        }

        if (unit() is Per<*, *>
            && other.unit().baseUnit == (unit() as Per<*, *>).denominator().baseUnit
        ) {
            // denominator of the Per cancels out, return with just the units of the numerator
            val numerator = (unit() as Per<*, *>).numerator()
            return numerator.ofBaseUnits(baseUnitMagnitude() * other.baseUnitMagnitude())
        } else if (unit() is Velocity<*> && other.unit().baseUnit == Units.Seconds) {
            // Multiplying a velocity by a time, return the scalar unit (e.g. Distance)
            val numerator = (unit() as Velocity<*>).unit
            return numerator.ofBaseUnits(baseUnitMagnitude() * other.baseUnitMagnitude())
        } else if (other.unit() is Per<*, *>
            && unit().baseUnit == (other.unit() as Per<*, *>).denominator().baseUnit
        ) {
            val numerator = (other.unit() as Per<*, *>).numerator()
            return numerator.ofBaseUnits(baseUnitMagnitude() * other.baseUnitMagnitude())
        } else if (unit() is Per<*, *>
            && other.unit() is Per<*, *>
            && ((unit() as Per<*, *>)
                .denominator()
                .baseUnit == (other.unit() as Per<*, *>).numerator().baseUnit)
            && ((unit() as Per<*, *>)
                .numerator()
                .baseUnit
                    == (other.unit() as Per<*, *>).denominator().baseUnit)
        ) {
            // multiplying e.g. meters per second * milliseconds per foot
            // return a scalar
            return Units.Value.of(baseUnitMagnitude() * other.baseUnitMagnitude())
        }

        // Dimensional analysis fallthrough, do a basic unit multiplication
        return unit().mult(other.unit()).ofBaseUnits(baseUnitMagnitude() * other.baseUnitMagnitude())
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * `times(1 / divisor)`
     *
     * @param divisor the constant to divide by
     * @return the resulting measure
     * @see .times
     */
    operator fun div(divisor: Double): Measure<U> {
        return times(1 / divisor)
    }

    /**
     * Divides this measurement by another measure and performs some dimensional analysis to reduce
     * the units.
     *
     * @param U2 the type of the other measure to multiply by
     * @param other the unit to multiply by
     * @return the resulting measure
     */
    operator fun <U2 : Unit<U2>> div(other: Measure<U2>): Measure<*> {
        if (unit().baseUnit == other.unit().baseUnit) {
            return Units.Value.ofBaseUnits(baseUnitMagnitude() / other.baseUnitMagnitude())
        }
        if (other.unit() is Dimensionless) {
            return div(other.baseUnitMagnitude())
        }
        if (other.unit() is Velocity<*>) {
            val velocity = other.unit() as Velocity<*>
            if (velocity.unit.baseUnit == unit().baseUnit) {
                return times(velocity.reciprocal().ofBaseUnits(1 / other.baseUnitMagnitude()))
            }
        }
        if (other.unit() is Per<*, *>) {
            val per = other.unit() as Per<*, *>
            if (per.numerator().baseUnit == unit().baseUnit) {
                return times(per.reciprocal().ofBaseUnits(1 / other.baseUnitMagnitude()))
            }
        }
        return unit().per(other.unit()).ofBaseUnits(baseUnitMagnitude() / other.baseUnitMagnitude())
    }

    /**
     * Creates a velocity measure by dividing this one by a time period measure.
     *
     * ```
     * Meters.of(1).per(Second) // Measure<Velocity<Distance>>
     * ```
     *
     * @param period the time period to divide by.
     * @return the velocity result
     */
    infix fun per(period: Measure<Time>): Measure<Velocity<U>> {
        val newUnit = unit().per(period.unit())
        return ImmutableMeasure.ofBaseUnits(baseUnitMagnitude() / period.baseUnitMagnitude(), newUnit)
    }

    /**
     * Creates a relational measure equivalent to this one per some other unit.
     *
     * ```
     * Volts.of(1.05).per(Meter) // V/m, potential PID constant
     * ```
     *
     * @param U2          the type of the denominator unit
     * @param denominator the denominator unit being divided by
     * @return the relational measure
     */
    infix fun <U2 : Unit<U2>> per(denominator: U2): Measure<Per<U, U2>> {
        val newUnit = unit().per(denominator)
        return newUnit.of(magnitude())
    }

    /**
     * Creates a velocity measure equivalent to this one per a unit of time.
     *
     * ```
     * Radians.of(3.14).per(Second) // Velocity&lt;Angle&gt; equivalent to RadiansPerSecond.of(3.14)
     * ```
     *
     * @param time the unit of time
     * @return the velocity measure
     */
    infix fun per(time: Time): Measure<Velocity<U>> {
        val newUnit = unit().per(time)
        return newUnit.of(magnitude())
    }

    /**
     * Adds another measure to this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to add to this one
     * @return a new measure containing the result
     */
    operator fun plus(other: Measure<U>): Measure<U> {
        return unit().ofBaseUnits(baseUnitMagnitude() + other.baseUnitMagnitude())
    }

    /**
     * Subtracts another measure from this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to subtract from this one
     * @return a new measure containing the result
     */
    operator fun minus(other: Measure<U>): Measure<U> {
        return unit().ofBaseUnits(baseUnitMagnitude() - other.baseUnitMagnitude())
    }

    /**
     * Negates this measure and returns the result.
     *
     * @return the resulting measure
     */
    fun negate(): Measure<U> {
        return times(-1.0)
    }

    /**
     * Negates this measure and returns the result.
     *
     * @return the resulting measure
     */
    operator fun unaryMinus(): Measure<U> {
        return negate()
    }

    /**
     * Returns an immutable copy of this measure. The copy can be used freely and is guaranteed never
     * to change.
     *
     * @return the copied measure
     */
    fun copy(): Measure<U>

    /**
     * Creates a new mutable copy of this measure.
     *
     * @return a mutable measure initialized to be identical to this measure
     */
    fun mutableCopy(): MutableMeasure<U> {
        return MutableMeasure.mutable(this)
    }

    /**
     * Checks if this measure is near another measure of the same unit. Provide a variance threshold
     * for use for a +/- scalar, such as 0.05 for +/- 5%.
     *
     * ```
     * Inches.of(11).isNear(Inches.of(10), 0.1) // true
     * Inches.of(12).isNear(Inches.of(10), 0.1) // false
     * ```
     *
     * @param varianceThreshold the acceptable variance threshold, in terms of an acceptable +/- error
     * range multiplier. Checking if a value is within 10% means a value of 0.1 should be passed;
     * checking if a value is within 1% means a value of 0.01 should be passed, and so on.
     * @return true if this unit is near the other measure, otherwise false
     */
    infix fun Measure<*>.isNear(varianceThreshold: Double): Boolean {
        @Suppress("LABEL_RESOLVE_WILL_CHANGE")
        if (!this@Measure.unit().baseUnit.equivalent(this.unit().baseUnit)) {
            return false // Disjoint units, not compatible
        }

        // Absolute so negative inputs are calculated correctly
        val tolerance = abs(this.baseUnitMagnitude() * varianceThreshold)

        @Suppress("LABEL_RESOLVE_WILL_CHANGE")
        return abs(this@Measure.baseUnitMagnitude() - this.baseUnitMagnitude()) <= tolerance
    }

    /**
     * Checks if this measure is near another measure of the same unit, with a specified tolerance of
     * the same unit.
     *
     * ```
     * Meters.of(1).isNear(Meters.of(1.2), Millimeters.of(300)) // true
     * Degrees.of(90).isNear(Rotations.of(0.5), Degrees.of(45)) // false
     * ```
     *
     * @param tolerance the tolerance allowed in which the two measures are defined as near each
     * other.
     * @return true if this unit is near the other measure, otherwise false.
     */
    infix fun Measure<U>.isNear(tolerance: Measure<U>): Boolean {
        @Suppress("LABEL_RESOLVE_WILL_CHANGE")
        return abs(this@Measure.baseUnitMagnitude() - this.baseUnitMagnitude()) <= abs(tolerance.baseUnitMagnitude())
    }

    /**
     * Checks if this measure is equivalent to another measure of the same unit.
     *
     * @param other the measure to compare to
     * @return true if this measure is equivalent, false otherwise
     */
    infix fun isEquivalent(other: Measure<*>): Boolean {
        if (unit().baseUnit != other.unit().baseUnit) {
            return false // Disjoint units, not compatible
        }

        return abs(baseUnitMagnitude() - other.baseUnitMagnitude()) <= EQUIVALENCE_THRESHOLD
    }

    override infix fun compareTo(other: Measure<U>): Int {
        return baseUnitMagnitude().compareTo(other.baseUnitMagnitude())
    }

    /**
     * Checks if this measure is greater than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a greater equivalent magnitude, false otherwise
     */
    infix fun gt(o: Measure<U>): Boolean {
        return compareTo(o) > 0
    }

    /**
     * Checks if this measure is greater than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or greater equivalent magnitude, false otherwise
     */
    infix fun gte(o: Measure<U>): Boolean {
        return compareTo(o) > 0 || isEquivalent(o)
    }

    /**
     * Checks if this measure is less than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a lesser equivalent magnitude, false otherwise
     */
    infix fun lt(o: Measure<U>): Boolean {
        return compareTo(o) < 0
    }

    /**
     * Checks if this measure is less than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or lesser equivalent magnitude, false otherwise
     */
    infix fun lte(o: Measure<U>): Boolean {
        return compareTo(o) < 0 || isEquivalent(o)
    }

    /**
     * Returns a string representation of this measurement in a scientific shorthand form. The symbol of the
     * backing unit is used, rather than the full name, and the magnitude is represented in scientific
     * notation.
     *
     * @return the scientific shorthand form representation of this measurement
     */
    fun toScientificString(): String {
        // eg 1.234e+04 V/m (1234 Volt per Meter in long form)
        return String.format(Locale.getDefault(), "%.3e %s", magnitude(), unit().symbol())
    }

    /**
     * Returns a string representation of this measurement in the default form. The symbol of the
     * backing unit is used, rather than the full name, and the magnitude is represented in full, not in scientific
     * notation. (Very large values may be represented in scientific notation, however)
     *
     * @return the short/default form representation of this measurement
     */
    fun toShortString(): String {
        // BunyipsLib change: The old short string for most applications did not fit purpose, since all values in FTC
        // are fairly small quantity and large values are already represented in scientific notation. Short string
        // now gives the magnitude in full (old method exists in toScientificString()).

        // eg 1234 V/m (1.234e+04 V/m in scientific form)

        return String.format(Locale.getDefault(), "%s %s", magnitude(), unit().symbol())
    }

    /**
     * Returns a string representation of this measurement in a longhand form. The name of the backing
     * unit is used, rather than its symbol, and the magnitude is represented in a full string, not
     * scientific notation. (Very large values may be represented in scientific notation, however)
     *
     * @return the long form representation of this measurement
     */
    fun toLongString(): String {
        // eg 1234 Volt per Meter (1.234e+04 V/m in scientific form)
        return String.format("%s %s", magnitude(), unit().name())
    }

    companion object {
        /**
         * Returns the measure with the absolute value closest to positive infinity.
         *
         * @param U        the type of the units of the measures
         * @param measures the set of measures to compare
         * @return the measure with the greatest positive magnitude, or null if no measures were provided
         */
        @JvmStatic
        @SafeVarargs
        fun <U : Unit<U>> max(vararg measures: Measure<U>): Measure<U>? {
            if (measures.isEmpty()) {
                return null // nothing to compare
            }

            var max: Measure<U>? = null
            for (measure in measures) {
                if (max == null || measure.gt(max)) {
                    max = measure
                }
            }

            return max
        }

        /**
         * Returns the measure with the absolute value closest to negative infinity.
         *
         * @param U        the type of the units of the measures
         * @param measures the set of measures to compare
         * @return the measure with the greatest negative magnitude
         */
        @JvmStatic
        @SafeVarargs
        fun <U : Unit<U>> min(vararg measures: Measure<U>): Measure<U>? {
            if (measures.isEmpty()) {
                return null // nothing to compare
            }

            var max: Measure<U>? = null
            for (measure in measures) {
                if (max == null || measure.lt(max)) {
                    max = measure
                }
            }

            return max
        }

        /**
         * The threshold for two measures to be considered equivalent if converted to the same unit. This
         * is only needed due to floating-point error.
         */
        const val EQUIVALENCE_THRESHOLD: Double = 1.0e-12
    }
}
