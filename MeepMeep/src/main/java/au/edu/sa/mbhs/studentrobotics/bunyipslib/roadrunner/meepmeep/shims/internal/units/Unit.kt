// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.ImmutableMeasure.Companion.ofRelativeUnits
import java.util.Objects
import kotlin.math.abs

/**
 * Unit of measurement that defines a quantity, such as grams, meters, or seconds.
 *
 * This is the base class for units. Actual units (such as [Units.Grams] and [ ][Units.Meters]) can be found in the [Units] class.
 *
 * @param <U> the self type, e.g. `class SomeUnit extends Unit<SomeUnit>`
 * @since 1.0.0-pre
 */
open class Unit<U : Unit<U>>(
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
    @JvmField
    val baseUnit: U,
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
    protected constructor(baseUnit: U, baseUnitEquivalent: Double, name: String, symbol: String) : this(
        baseUnit,
        { x: Double -> x * baseUnitEquivalent },
        { x: Double -> x / baseUnitEquivalent }, name, symbol
    )

    /**
     * Checks if this unit is the base unit for its own system of measurement.
     *
     * @return true if this is the base unit, false if not
     */
    fun isBaseUnit(): Boolean {
        return equals(baseUnit)
    }

    /**
     * Converts a value in terms of base units to a value in terms of this unit.
     *
     * @param valueInBaseUnits the value in base units to convert
     * @return the equivalent value in terms of this unit
     */
    fun fromBaseUnits(valueInBaseUnits: Double): Double {
        return converterFromBase.apply(valueInBaseUnits)
    }

    /**
     * Converts a value in terms of this unit to a value in terms of the base unit.
     *
     * @param valueInNativeUnits the value in terms of this unit to convert
     * @return the equivalent value in terms of the base unit
     */
    fun toBaseUnits(valueInNativeUnits: Double): Double {
        return converterToBase.apply(valueInNativeUnits)
    }

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
    fun convertFrom(magnitude: Double, otherUnit: Unit<U>): Double {
        if (equivalent(otherUnit)) {
            // same unit, don't bother converting
            return magnitude
        }
        return fromBaseUnits(otherUnit.toBaseUnits(magnitude))
    }

    /**
     * Creates a new measure of this unit with the given value. The resulting measure is
     * *immutable* and cannot have its value modified.
     *
     * @param magnitude the magnitude of the measure to create
     * @return the measure
     */
    infix fun of(magnitude: Double): Measure<U> {
        if (magnitude == 0.0) {
            // reuse static object
            return zero()
        }
        if (magnitude == 1.0) {
            // reuse static object
            return one()
        }
        return ofRelativeUnits(
            magnitude,
            this
        )
    }

    companion object {
        /**
         * Creates a new measure of this unit with the given value. The resulting measure is
         * *immutable* and cannot have its value modified.
         *
         * @return the measure object as associated with the given number
         */
        infix fun <U : Unit<U>> Number.of(unit: Unit<U>): Measure<U> {
            return unit.of(this.toDouble())
        }
    }

    /**
     * Creates a new measure with a magnitude equal to the given base unit magnitude, converted to be
     * in terms of this unit.
     *
     * @param baseUnitMagnitude the magnitude of the measure in terms of the base unit
     * @return the measure
     */
    infix fun ofBaseUnits(baseUnitMagnitude: Double): Measure<U> {
        return ImmutableMeasure.ofBaseUnits(
            baseUnitMagnitude,
            this
        )
    }

    /**
     * Gets a measure with a magnitude of 0 in terms of this unit.
     *
     * @return the zero-valued measure
     */
    fun zero(): Measure<U> {
        // lazy init because 'this' is null in object initialization
        if (zero == null) {
            zero = ofRelativeUnits(
                0.0,
                this
            )
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
            one = ofRelativeUnits(
                1.0,
                this
            )
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
    infix fun per(period: Time): Velocity<U> {
        return Velocity.combine(this, period)
    }

    /**
     * Takes this unit and creates a new proportional unit where this unit is the numerator and the
     * given denominator is the denominator.
     *
     * ```
     * Volts.per(Meter) // V/m
     * ```
     *
     * @param <D>         the type of the denominator units
     * @param denominator the denominator of the proportional unit
     * @return a combined proportional unit
     */
    infix fun <D : Unit<D>> per(denominator: D): Per<U, D> {
        @Suppress("UNCHECKED_CAST")
        return Per.combine(this as U, denominator)
    }

    /**
     * Takes this unit and creates a new combinatory unit equivalent to this unit multiplied by
     * another.
     *
     * ```
     * Volts.mult(Meter) // V*m
     * ```
     *
     * @param <U2>  the type of the unit to multiply by
     * @param other the unit to multiply by
     * @return a combined unit equivalent to this unit multiplied by the other
    </U2> */
    infix fun <U2 : Unit<U2>> mult(other: U2): Mult<U, U2> {
        @Suppress("UNCHECKED_CAST")
        return Mult.combine(this as U, other)
    }

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

        return abs(
            converterFromBase.apply(arbitrary)
                    - other.converterFromBase.apply(arbitrary)
        ) <= Measure.EQUIVALENCE_THRESHOLD
                && abs(
            converterToBase.apply(arbitrary) - other.converterToBase.apply(arbitrary)
        ) <= Measure.EQUIVALENCE_THRESHOLD
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

    override fun hashCode(): Int {
        return Objects.hash(converterToBase, converterFromBase, name, symbol)
    }

    /**
     * Gets the name of this unit.
     *
     * @return the unit's name
     */
    fun name(): String {
        return name
    }

    /**
     * Gets the symbol of this unit.
     *
     * @return the unit's symbol
     */
    fun symbol(): String {
        return symbol
    }

    override fun toString(): String {
        return name()
    }
}
