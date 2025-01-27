// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units

import java.util.Objects

/**
 * A specialization of [Measure] that allows for mutability. This is intended to be used for
 * memory use reasons and should NOT be exposed in the public API for a class that uses it.
 *
 * The advantage of using this class is to reuse one instance of a measurement object, as opposed
 * to instantiating a new immutable instance every time an operation is performed. This will reduce
 * memory pressure, but comes at the cost of increased code complexity and sensitivity to race
 * conditions if misused.
 *
 * Any unsafe methods are prefixed with `mut_*`, such as [mut_plus] or
 * [mut_replace]. These methods will change the internal state of the measurement
 * object, and as such can be dangerous to use. They are primarily intended for use to track
 * internal state of things like sensors
 *
 * @param U the type of the unit of measure
 * @since 1.0.0-pre
 */
@Suppress("FunctionName")
class MutableMeasure<U : Unit<U>> private constructor(
    private var magnitudeVal: Double,
    private var baseUnitMagnitudeVal: Double,
    private var unitVal: U
) : Measure<U> {
    override fun magnitude() = magnitudeVal

    override fun baseUnitMagnitude() = baseUnitMagnitudeVal

    override fun unit() = unitVal

    // UNSAFE
    /**
     * Sets the new magnitude of the measurement. The magnitude must be in terms of the unit.
     *
     * @param magnitude the new magnitude of the measurement
     */
    fun mut_setMagnitude(magnitude: Number) {
        magnitudeVal = magnitude.toDouble()
        baseUnitMagnitudeVal = unitVal.toBaseUnits(magnitude)
    }

    /**
     * Sets the new magnitude of the measurement. The magnitude must be in terms of the base unit of
     * the current unit.
     *
     * @param baseUnitMagnitude the new magnitude of the measurement
     */
    fun mut_setBaseUnitMagnitude(baseUnitMagnitude: Number) {
        baseUnitMagnitudeVal = baseUnitMagnitude.toDouble()
        magnitudeVal = unitVal.fromBaseUnits(baseUnitMagnitude)
    }

    /**
     * Overwrites the state of this measure and replaces it with values from the given one.
     *
     * @param other the other measure to copy values from
     * @return this measure
     */
    fun mut_replace(other: Measure<U>) = apply {
        magnitudeVal = other.magnitude()
        baseUnitMagnitudeVal = other.baseUnitMagnitude()
        unitVal = other.unit()
    }

    /**
     * Overwrites the state of this measure with new values.
     *
     * @param magnitude the new magnitude in terms of the new unit
     * @param unit      the new unit
     * @return this measure
     */
    fun mut_replace(magnitude: Number, unit: U) = apply {
        magnitudeVal = magnitude.toDouble()
        baseUnitMagnitudeVal = unit.toBaseUnits(magnitude)
        unitVal = unit
    }

    /**
     * Increments the current magnitude of the measure by the given value. The value must be in terms
     * of the current unit.
     *
     * @param raw the raw value to accumulate by
     * @return the measure
     */
    fun mut_acc(raw: Number) = apply {
        magnitudeVal += raw.toDouble()
        baseUnitMagnitudeVal += unitVal.toBaseUnits(raw)
    }

    /**
     * Increments the current magnitude of the measure by the amount of the given measure.
     *
     * @param other the measure whose value should be added to this one
     * @return the measure
     */
    fun mut_acc(other: Measure<U>) = apply {
        baseUnitMagnitudeVal += other.baseUnitMagnitude()

        // can't naively use magnitude += other.in(unit) because the units may not
        // be scalar multiples (e.g. adding 0C to 100K should result in 373.15K, not 100K)
        magnitudeVal = unitVal.fromBaseUnits(baseUnitMagnitudeVal)
    }

    // Math
    /**
     * Adds another measurement to this one. This will mutate the object instead of generating a new
     * measurement object.
     *
     * @param other the measurement to add
     * @return this measure
     */
    fun mut_plus(other: Measure<U>) = mut_plus(other.magnitude(), other.unit())

    /**
     * Adds another measurement to this one. This will mutate the object instead of generating a new
     * measurement object. This is a denormalized version of [mut_plus] to avoid
     * having to wrap raw numbers in a `Measure` object and pay for an object allocation.
     *
     * @param magnitude the magnitude of the other measurement.
     * @param unit      the unit of the other measurement
     * @return this measure
     */
    fun mut_plus(magnitude: Number, unit: U) = apply {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal + unit.toBaseUnits(magnitude))
    }

    /**
     * Subtracts another measurement to this one. This will mutate the object instead of generating a
     * new measurement object.
     *
     * @param other the measurement to add
     * @return this measure
     */
    fun mut_minus(other: Measure<U>) = mut_minus(other.magnitude(), other.unit())

    /**
     * Subtracts another measurement to this one. This will mutate the object instead of generating a
     * new measurement object. This is a denormalized version of [mut_minus] to avoid
     * having to wrap raw numbers in a `Measure` object and pay for an object allocation.
     *
     * @param magnitude the magnitude of the other measurement.
     * @param unit      the unit of the other measurement
     * @return this measure
     */
    fun mut_minus(magnitude: Number, unit: U) = mut_plus(-magnitude.toDouble(), unit)

    /**
     * Multiplies this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param multiplier the multiplier to scale the measurement by
     * @return this measure
     */
    fun mut_times(multiplier: Number) = apply {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal * multiplier.toDouble())
    }

    /**
     * Multiplies this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param multiplier the multiplier to scale the measurement by
     * @return this measure
     */
    fun mut_times(multiplier: Measure<out Dimensionless>) = mut_times(multiplier.baseUnitMagnitude())

    /**
     * Divides this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param divisor the divisor to scale the measurement by
     * @return this measure
     */
    fun mut_divide(divisor: Number) = apply {
        mut_setBaseUnitMagnitude(baseUnitMagnitudeVal / divisor.toDouble())
    }

    /**
     * Divides this measurement by some constant value. This will mutate the object instead of
     * generating a new measurement object.
     *
     * @param divisor the divisor to scale the measurement by
     * @return this measure
     */
    fun mut_divide(divisor: Measure<out Dimensionless>) = mut_divide(divisor.baseUnitMagnitude())

    override fun copy() = ImmutableMeasure(magnitudeVal, baseUnitMagnitudeVal, unitVal)

    override fun toString() = toShortString()

    override fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other !is Measure<*>) {
            return false
        }
        return unitVal == other.unit() && isEquivalent(other)
    }

    override fun hashCode() = Objects.hash(magnitudeVal, unitVal)

    companion object {
        /**
         * Creates a new mutable measure that is a copy of the given one.
         *
         * @param U       the type of the units of measure
         * @param measure the measure to create a mutable copy of
         * @return a new mutable measure with an initial state equal to the given measure
         */
        @JvmStatic
        fun <U : Unit<U>> mutable(measure: Measure<U>) =
            MutableMeasure(measure.magnitude(), measure.baseUnitMagnitude(), measure.unit())

        /**
         * Creates a new mutable measure with a magnitude of 0 in the given unit.
         *
         * @param U    the type of the units of measure
         * @param unit the unit of measure
         * @return a new mutable measure
         */
        @JvmStatic
        fun <U : Unit<U>> zero(unit: U) = mutable(unit.zero())

        /**
         * Creates a new mutable measure in the given unit with a magnitude equal to the given one in base
         * units.
         *
         * @param U                 the type of the units of measure
         * @param baseUnitMagnitude the magnitude of the measure, in terms of the base unit of measure
         * @param unit              the unit of measure
         * @return a new mutable measure
         */
        @JvmStatic
        fun <U : Unit<U>> ofBaseUnits(baseUnitMagnitude: Number, unit: U) =
            MutableMeasure(unit.fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude.toDouble(), unit)

        /**
         * Creates a new mutable measure in the given unit with a magnitude in terms of that unit.
         *
         * @param U                 the type of the units of measure
         * @param relativeMagnitude the magnitude of the measure
         * @param unit              the unit of measure
         * @return a new mutable measure
         */
        @JvmStatic
        fun <U : Unit<U>> ofRelativeUnits(relativeMagnitude: Number, unit: U) =
            MutableMeasure(relativeMagnitude.toDouble(), unit.toBaseUnits(relativeMagnitude), unit)
    }
}
