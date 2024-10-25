// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units

import java.util.Objects

/**
 * A specialization of [Measure] that allows for mutability. This is intended to be used for
 * memory use reasons (such as on the memory-restricted roboRIO 1 or 2 or SBC coprocessors) and
 * should NOT be exposed in the public API for a class that uses it.
 *
 *
 * The advantage of using this class is to reuse one instance of a measurement object, as opposed
 * to instantiating a new immutable instance every time an operation is performed. This will reduce
 * memory pressure, but comes at the cost of increased code complexity and sensitivity to race
 * conditions if misused.
 *
 *
 * Any unsafe methods are prefixed with `mut_*`, such as [mut_plus] or
 * [mut_replace]. These methods will change the internal state of the measurement
 * object, and as such can be dangerous to use. They are primarily intended for use to track
 * internal state of things like sensors
 *
 * @param <U> the type of the unit of measure
 * @since 1.0.0-pre
 */
@Suppress("FunctionName")
class MutableMeasure<U : Unit<U>> private constructor(
    private var magnitudeVal: Double,
    private var baseUnitMagnitudeVal: Double,
    private var unitVal: U
) : Measure<U> {
    override fun magnitude(): Double {
        return magnitudeVal
    }

    override fun baseUnitMagnitude(): Double {
        return baseUnitMagnitudeVal
    }

    override fun unit(): U {
        return unitVal
    }

    // UNSAFE
    /**
     * Sets the new magnitude of the measurement. The magnitude must be in terms of the unit.
     *
     * @param magnitude the new magnitude of the measurement
     */
    fun mut_setMagnitude(magnitude: Double) {
        magnitudeVal = magnitude
        baseUnitMagnitudeVal = unitVal.toBaseUnits(magnitude)
    }

    /**
     * Sets the new magnitude of the measurement. The magnitude must be in terms of the base unit of
     * the current unit.
     *
     * @param baseUnitMagnitude the new magnitude of the measurement
     */
    fun mut_setBaseUnitMagnitude(baseUnitMagnitude: Double) {
        baseUnitMagnitudeVal = baseUnitMagnitude
        magnitudeVal = unitVal.fromBaseUnits(baseUnitMagnitude)
    }

    /**
     * Overwrites the state of this measure and replaces it with values from the given one.
     *
     * @param other the other measure to copy values from
     * @return this measure
     */
    fun mut_replace(other: Measure<U>): MutableMeasure<U> {
        magnitudeVal = other.magnitude()
        baseUnitMagnitudeVal = other.baseUnitMagnitude()
        unitVal = other.unit()
        return this
    }

    /**
     * Overwrites the state of this measure with new values.
     *
     * @param magnitude the new magnitude in terms of the new unit
     * @param unit      the new unit
     * @return this measure
     */
    fun mut_replace(magnitude: Double, unit: U): MutableMeasure<U> {
        magnitudeVal = magnitude
        baseUnitMagnitudeVal = unit.toBaseUnits(magnitude)
        unitVal = unit
        return this
    }

    /**
     * Increments the current magnitude of the measure by the given value. The value must be in terms
     * of the current unit.
     *
     * @param raw the raw value to accumulate by
     * @return the measure
     */
    fun mut_acc(raw: Double): MutableMeasure<U> {
        magnitudeVal += raw
        baseUnitMagnitudeVal += unitVal.toBaseUnits(raw)
        return this
    }

    /**
     * Increments the current magnitude of the measure by the amount of the given measure.
     *
     * @param other the measure whose value should be added to this one
     * @return the measure
     */
    fun mut_acc(other: Measure<U>): MutableMeasure<U> {
        baseUnitMagnitudeVal += other.baseUnitMagnitude()

        // can't naively use magnitude += other.in(unit) because the units may not
        // be scalar multiples (eg adding 0C to 100K should result in 373.15K, not 100K)
        magnitudeVal = unitVal.fromBaseUnits(baseUnitMagnitudeVal)
        return this
    }

    // Math
    /**
     * Adds another measurement to this one. This will mutate the object instead of generating a new
     * measurement object.
     *
     * @param other the measurement to add
     * @return this measure
     */
    fun mut_plus(other: Measure<U>): MutableMeasure<U> {
        return mut_plus(other.magnitude(), other.unit())
    }

    /**
     * Adds another measurement to this one. This will mutate the object instead of generating a new
     * measurement object. This is a denormalized version of [mut_plus] to avoid
     * having to wrap raw numbers in a `Measure` object and pay for an object allocation.
     *
     * @param magnitude the magnitude of the other measurement.
     * @param unit      the unit of the other measurement
     * @return this measure
     */
    fun mut_plus(magnitude: Double, unit: U): MutableMeasure<U> {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal + unit.toBaseUnits(magnitude))
        return this
    }

    /**
     * Subtracts another measurement to this one. This will mutate the object instead of generating a
     * new measurement object.
     *
     * @param other the measurement to add
     * @return this measure
     */
    fun mut_minus(other: Measure<U>): MutableMeasure<U> {
        return mut_minus(other.magnitude(), other.unit())
    }

    /**
     * Subtracts another measurement to this one. This will mutate the object instead of generating a
     * new measurement object. This is a denormalized version of [mut_minus] to avoid
     * having to wrap raw numbers in a `Measure` object and pay for an object allocation.
     *
     * @param magnitude the magnitude of the other measurement.
     * @param unit      the unit of the other measurement
     * @return this measure
     */
    fun mut_minus(magnitude: Double, unit: U): MutableMeasure<U> {
        return mut_plus(-magnitude, unit)
    }

    /**
     * Multiplies this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param multiplier the multiplier to scale the measurement by
     * @return this measure
     */
    fun mut_times(multiplier: Double): MutableMeasure<U> {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal * multiplier)
        return this
    }

    /**
     * Multiplies this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param multiplier the multiplier to scale the measurement by
     * @return this measure
     */
    fun mut_times(multiplier: Measure<out Dimensionless>): MutableMeasure<U> {
        return mut_times(multiplier.baseUnitMagnitude())
    }

    /**
     * Divides this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param divisor the divisor to scale the measurement by
     * @return this measure
     */
    fun mut_divide(divisor: Double): MutableMeasure<U> {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal / divisor)
        return this
    }

    /**
     * Divides this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param divisor the divisor to scale the measurement by
     * @return this measure
     */
    fun mut_divide(divisor: Measure<out Dimensionless>): MutableMeasure<U> {
        return mut_divide(divisor.baseUnitMagnitude())
    }

    override fun copy(): Measure<U> {
        return ImmutableMeasure(magnitudeVal, baseUnitMagnitudeVal, unitVal)
    }

    override fun toString(): String {
        return toShortString()
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other !is Measure<*>) {
            return false
        }
        return unitVal == other.unit() && isEquivalent(other)
    }

    override fun hashCode(): Int {
        return Objects.hash(magnitudeVal, unitVal)
    }

    companion object {
        /**
         * Creates a new mutable measure that is a copy of the given one.
         *
         * @param <U>     the type of the units of measure
         * @param measure the measure to create a mutable copy of
         * @return a new mutable measure with an initial state equal to the given measure
         */
        fun <U : Unit<U>> mutable(measure: Measure<U>): MutableMeasure<U> {
            return MutableMeasure(measure.magnitude(), measure.baseUnitMagnitude(), measure.unit())
        }

        /**
         * Creates a new mutable measure with a magnitude of 0 in the given unit.
         *
         * @param <U>  the type of the units of measure
         * @param unit the unit of measure
         * @return a new mutable measure
         */
        fun <U : Unit<U>> zero(unit: U): MutableMeasure<U> {
            return mutable(unit.zero())
        }

        /**
         * Creates a new mutable measure in the given unit with a magnitude equal to the given one in base
         * units.
         *
         * @param <U>               the type of the units of measure
         * @param baseUnitMagnitude the magnitude of the measure, in terms of the base unit of measure
         * @param unit              the unit of measure
         * @return a new mutable measure
         */
        fun <U : Unit<U>> ofBaseUnits(
            baseUnitMagnitude: Double, unit: U
        ): MutableMeasure<U> {
            return MutableMeasure(unit.fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, unit)
        }

        /**
         * Creates a new mutable measure in the given unit with a magnitude in terms of that unit.
         *
         * @param <U>               the type of the units of measure
         * @param relativeMagnitude the magnitude of the measure
         * @param unit              the unit of measure
         * @return a new mutable measure
         */
        fun <U : Unit<U>> ofRelativeUnits(
            relativeMagnitude: Double, unit: U
        ): MutableMeasure<U> {
            return MutableMeasure(relativeMagnitude, unit.toBaseUnits(relativeMagnitude), unit)
        }
    }
}
