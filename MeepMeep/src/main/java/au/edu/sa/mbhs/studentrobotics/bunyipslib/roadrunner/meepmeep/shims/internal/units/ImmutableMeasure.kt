// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units

import java.util.Objects

/**
 * A measure holds the magnitude and unit of some dimension, such as distance, time, or speed. An
 * immutable measure is *immutable* and *type safe*, making it easy to use in concurrent
 * situations and gives compile-time safety. Two measures with the same *unit* and
 * *magnitude* are effectively equivalent objects.
 *
 * @param <U> the unit type of the measure
 * @since 1.0.0-pre
 */
class ImmutableMeasure<U : Unit<U>> internal constructor(magnitude: Double, baseUnitMagnitude: Double, unit: Unit<U>) :
    Measure<U> {
    private val magnitude: Double
    private val baseUnitMagnitude: Double
    private val unit: U

    /**
     * Creates a new immutable measure instance. This shouldn't be used directly; prefer one of the
     * factory methods instead.
     */
    init {
        Objects.requireNonNull(unit, "Unit cannot be null")
        this.magnitude = magnitude
        this.baseUnitMagnitude = baseUnitMagnitude
        @Suppress("UNCHECKED_CAST")
        this.unit = unit as U
    }

    /**
     * Gets the unitless magnitude of this measure.
     */
    override fun magnitude(): Double {
        return magnitude
    }

    override fun baseUnitMagnitude(): Double {
        return baseUnitMagnitude
    }

    /**
     * Gets the units of this measure.
     */
    override fun unit(): U {
        return unit
    }

    /**
     * Checks for *object equality*. To check if two measures are *equivalent*, use [isEquivalent].
     */
    override fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other !is Measure<*>) {
            return false
        }
        return unit == other.unit() && baseUnitMagnitude == other.baseUnitMagnitude()
    }

    override fun hashCode(): Int {
        return Objects.hash(magnitude, unit)
    }

    override fun copy(): Measure<U> {
        return this // Already immutable, no need to allocate a new object
    }

    override fun toString(): String {
        return toShortString()
    }

    companion object {
        /**
         * Creates a new measure in the given unit with a magnitude equal to the given one in base units.
         *
         * @param <U>               the type of the units of measure
         * @param baseUnitMagnitude the magnitude of the measure, in terms of the base unit of measure
         * @param unit              the unit of measure
         * @return a new measure
         */
        fun <U : Unit<U>> ofBaseUnits(
            baseUnitMagnitude: Double, unit: Unit<U>
        ): ImmutableMeasure<U> {
            return ImmutableMeasure(unit.fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, unit)
        }

        /**
         * Creates a new measure in the given unit with a magnitude in terms of that unit.
         *
         * @param <U>               the type of the units of measure
         * @param relativeMagnitude the magnitude of the measure
         * @param unit              the unit of measure
         * @return a new measure
         */
        fun <U : Unit<U>> ofRelativeUnits(
            relativeMagnitude: Double, unit: Unit<U>
        ): ImmutableMeasure<U> {
            return ImmutableMeasure(relativeMagnitude, unit.toBaseUnits(relativeMagnitude), unit)
        }
    }
}
