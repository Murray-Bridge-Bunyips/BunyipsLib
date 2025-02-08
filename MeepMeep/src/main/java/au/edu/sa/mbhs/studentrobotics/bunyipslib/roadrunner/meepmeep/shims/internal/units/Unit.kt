// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.ImmutableMeasure.Companion.ofBaseUnits
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.ImmutableMeasure.Companion.ofRelativeUnits
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Grams
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Meters
import java.util.Objects
import kotlin.math.abs

/**
 * Unit of measurement that defines a quantity, such as grams, meters, or seconds.
 *
 * This is the base class for units. Actual units (such as [Grams] and [Meters]) can be found in the [Units] class.
 *
 * @param U the self type, e.g. `class SomeUnit extends Unit<SomeUnit>`
 * @since 1.0.0-pre
 */
open class Unit<U : Unit<U>>(
    baseUnit: U?,
    toBaseConverter: UnaryFunction,
    fromBaseConverter: UnaryFunction,
    name: String,
    symbol: String
) {
    /**
     * Gets the conversion function used to convert values to base unit terms. This generally
     * shouldn't need to be used directly; prefer [toBaseUnits] instead.
     *
     * @return the conversion function
     */
    val converterToBase: UnaryFunction = Objects.requireNonNull(toBaseConverter)

    /**
     * Gets the conversion function used to convert values to terms of this unit. This generally
     * shouldn't need to be used directly; prefer [fromBaseUnits] instead.
     *
     * @return the conversion function
     */
    val converterFromBase: UnaryFunction = Objects.requireNonNull(fromBaseConverter)

    /**
     * Gets the base unit of measurement that this unit is derived from. If the unit is the base unit,
     * the unit will be returned.
     *
     * ```
     * Unit baseUnit = new Unit(null, ...);
     * baseUnit.getBaseUnit(); // returns baseUnit
     * Unit derivedUnit = new Unit(baseUnit, ...);
     * derivedUnit.getBaseUnit(); // returns baseUnit
     * ```
     *
     * @return the base unit
     */
    @Suppress("LeakingThis", "UNCHECKED_CAST")
    @JvmField
    val baseUnit: U = baseUnit ?: this as U

    private val name: String = Objects.requireNonNull(name)
    private val symbol: String = Objects.requireNonNull(symbol)
    private var zero: Measure<U>? = null
    private var one: Measure<U>? = null

    /**
     * Creates a new unit with the given name and multiplier to the base unit.
     *
     * @param baseUnit           the base unit, e.g. Meters for distances
     * @param baseUnitEquivalent the multiplier to convert this unit to the base unit of this type.
     * For example, meters has a multiplier of 1, mm has a multiplier of 1e3, and km has
     * multiplier of 1e-3.
     * @param name               the name of the unit. This should be a singular noun (so "Meter", not "Meters")
     * @param symbol             the short symbol for the unit, such as "m" for meters or "lb." for pounds
     */
    protected constructor(baseUnit: U?, baseUnitEquivalent: Double, name: String, symbol: String) : this(
        baseUnit, { it * baseUnitEquivalent }, { it / baseUnitEquivalent }, name, symbol
    )

    /**
     * Checks if this unit is the base unit for its own system of measurement.
     *
     * @return true if this is the base unit, false if not
     */
    fun isBaseUnit() = equals(baseUnit)

    /**
     * Converts a value in terms of base units to a value in terms of this unit.
     *
     * @param valueInBaseUnits the value in base units to convert
     * @return the equivalent value in terms of this unit
     */
    fun fromBaseUnits(valueInBaseUnits: Number) = converterFromBase.apply(valueInBaseUnits.toDouble())

    /**
     * Converts a value in terms of this unit to a value in terms of the base unit.
     *
     * @param valueInNativeUnits the value in terms of this unit to convert
     * @return the equivalent value in terms of the base unit
     */
    fun toBaseUnits(valueInNativeUnits: Number) = converterToBase.apply(valueInNativeUnits.toDouble())

    /**
     * Converts a magnitude in terms of another unit of the same dimension to a magnitude in terms of
     * this unit.
     *
     * ```
     * Inches.convertFrom(12, Feet) // 144.0
     * Kilograms.convertFrom(2.2, Pounds) // 0.9979024
     * ```
     *
     * @param magnitude a magnitude measured in another unit
     * @param otherUnit the unit to convert the magnitude to
     * @return the corresponding value in terms of this unit.
     */
    fun convertFrom(magnitude: Number, otherUnit: Unit<U>): Double {
        val m = magnitude.toDouble()
        if (equivalent(otherUnit)) {
            // same unit, don't bother converting
            return m
        }
        return fromBaseUnits(otherUnit.toBaseUnits(m))
    }

    /**
     * Creates a new measure of this unit with the given value. The resulting measure is
     * *immutable* and cannot have its value modified.
     *
     * @param magnitude the magnitude of the measure to create
     * @return the measure
     */
    infix fun of(magnitude: Number): Measure<U> {
        val m = magnitude.toDouble()
        if (m == 0.0) {
            // reuse static object
            return zero()
        }
        if (m == 1.0) {
            // reuse static object
            return one()
        }
        return ofRelativeUnits(m, this)
    }

    companion object {
        /**
         * Creates a new measure of this unit with the given value. The resulting measure is
         * *immutable* and cannot have its value modified.
         *
         * ```
         * 15 of Seconds
         * ```
         *
         * @return the measure object as associated with the given number
         */
        infix fun <U : Unit<U>> Number.of(unit: Unit<U>) = unit.of(this.toDouble())

        /**
         * Converts a magnitude in terms of another unit of the same dimension to a magnitude in terms of
         * this unit.
         *
         * ```
         * Inches from (12 to Feet) // 144.0
         * Kilograms from (2.2 to Pounds) // 0.9979024
         * ```
         *
         * @param conversion a pair of the magnitude and the unit to convert to
         * @return the corresponding value in terms of the other unit.
         */
        infix fun <U : Unit<U>> Unit<U>.from(conversion: Pair<Number, Unit<U>>) =
            convertFrom(conversion.first.toDouble(), conversion.second)
    }

    /**
     * Creates a new measure with a magnitude equal to the given base unit magnitude, converted to be
     * in terms of this unit.
     *
     * @param baseUnitMagnitude the magnitude of the measure in terms of the base unit
     * @return the measure
     */
    infix fun ofBaseUnits(baseUnitMagnitude: Number) = ofBaseUnits(baseUnitMagnitude.toDouble(), this)

    /**
     * Gets a measure with a magnitude of 0 in terms of this unit.
     *
     * @return the zero-valued measure
     */
    fun zero(): Measure<U> {
        // lazy init because 'this' is null in object initialization
        if (zero == null) {
            zero = ofRelativeUnits(0.0, this)
        }
        return zero as Measure<U>
    }

    /**
     * Gets a measure with a magnitude of 1 in terms of this unit.
     *
     * @return the 1-valued measure
     */
    fun one(): Measure<U> {
        // lazy init because 'this' is null in object initialization
        if (one == null) {
            one = ofRelativeUnits(1.0, this)
        }
        return one as Measure<U>
    }

    /**
     * Creates a velocity unit derived from this one. Can be chained to denote velocity, acceleration,
     * jerk, etc.
     *
     * ```
     * Meters.per(Second) // linear velocity
     * Kilograms.per(Second) // mass flow
     * Feet.per(Second).per(Second).of(32) // roughly 1G of acceleration
     * ```
     *
     * @param period the time period of the velocity, such as seconds or milliseconds
     * @return a velocity unit corresponding to the rate of change of this unit over time
     */
    infix fun per(period: Time) = Velocity.combine(this, period)

    /**
     * Creates a velocity unit derived from this one. Can be chained to denote velocity, acceleration,
     * jerk, etc.
     *
     * ```
     * Meters / Second // linear velocity
     * Kilograms / Second // mass flow
     * 32 of (Feet / Second / Second) // roughly 1G of acceleration
     * ```
     *
     * @param period the time period of the velocity, such as seconds or milliseconds
     * @return a velocity unit corresponding to the rate of change of this unit over time
     */
    @JvmName("perKt")
    operator fun div(period: Time) = per(period)

    /**
     * Takes this unit and creates a new proportional unit where this unit is the numerator and the
     * given denominator is the denominator.
     *
     * ```
     * Volts.per(Meter) // V/m
     * ```
     *
     * @param D           the type of the denominator units
     * @param denominator the denominator of the proportional unit
     * @return a combined proportional unit
     */
    @Suppress("UNCHECKED_CAST")
    infix fun <D : Unit<D>> per(denominator: D) = Per.combine(this as U, denominator)

    /**
     * Takes this unit and creates a new proportional unit where this unit is the numerator and the
     * given denominator is the denominator.
     *
     * ```
     * Volts / Meter // V/m
     * ```
     *
     * @param D           the type of the denominator units
     * @param denominator the denominator of the proportional unit
     * @return a combined proportional unit
     */
    @JvmName("perKt")
    operator fun <D : Unit<D>> div(denominator: D) = per(denominator)

    /**
     * Takes this unit and creates a new combinatory unit equivalent to this unit multiplied by
     * another.
     *
     * ```
     * Volts.mult(Meter) // V*m
     * ```
     *
     * @param U2    the type of the unit to multiply by
     * @param other the unit to multiply by
     * @return a combined unit equivalent to this unit multiplied by the other
     */
    @Suppress("UNCHECKED_CAST")
    infix fun <U2 : Unit<U2>> mult(other: U2) = Mult.combine(this as U, other)

    /**
     * Takes this unit and creates a new combinatory unit equivalent to this unit multiplied by
     * another.
     *
     * ```
     * Volts * Meter // V*m
     * ```
     *
     * @param U2    the type of the unit to multiply by
     * @param other the unit to multiply by
     * @return a combined unit equivalent to this unit multiplied by the other
     */
    @JvmName("multKt")
    operator fun <U2 : Unit<U2>> times(other: U2) = mult(other)

    /**
     * Checks if this unit is equivalent to another one. Equivalence is determined by both units
     * having the same base type and treat the same base unit magnitude as the same magnitude in their
     * own units, to within [Measure.EQUIVALENCE_THRESHOLD].
     *
     * @param other the unit to compare to.
     * @return true if both units are equivalent, false if not
     */
    infix fun equivalent(other: Unit<*>): Boolean {
        if (javaClass != other.javaClass) {
            // different unit types, not compatible
            return false
        }

        val arbitrary = 16777.214 // 2^24 / 1e3

        return abs(converterFromBase.apply(arbitrary) - other.converterFromBase.apply(arbitrary)) <= Measure.EQUIVALENCE_THRESHOLD &&
                abs(converterToBase.apply(arbitrary) - other.converterToBase.apply(arbitrary)) <= Measure.EQUIVALENCE_THRESHOLD
    }

    override operator fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other !is Unit<*>) {
            return false
        }
        return name == other.name && symbol == other.symbol && equivalent(other)
    }

    override fun hashCode() = Objects.hash(converterToBase, converterFromBase, name, symbol)

    /**
     * Gets the name of this unit.
     *
     * @return the unit's name
     */
    fun name() = name

    /**
     * Gets the symbol of this unit.
     *
     * @return the unit's symbol
     */
    fun symbol() = symbol

    override fun toString() = name()
}
