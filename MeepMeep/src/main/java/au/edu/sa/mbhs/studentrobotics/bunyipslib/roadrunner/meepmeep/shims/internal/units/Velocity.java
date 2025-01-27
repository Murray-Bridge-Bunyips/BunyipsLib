// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units;

import androidx.annotation.NonNull;

import java.util.Objects;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.collections.LongToObjectHashMap;

/**
 * Unit of velocity dimension that is a combination of a distance unit (numerator) and a time unit
 * (denominator).
 *
 * <p>This is the base type for units of velocity dimension. It is also used in combination with a
 * distance dimension to specify the dimension for {@link Measure}. For example: {@code
 * Measure<Velocity<Distance>>}.
 *
 * <p>Actual units (such as {@link Units#MetersPerSecond} and {@link Units#RPM}) can be found in the
 * {@link Units} class.
 *
 * @param <D> the distance unit, such as {@link Angle} or {@link Distance}
 * @since 1.0.0-pre
 */
public class Velocity<D extends Unit<D>> extends Unit<Velocity<D>> {
    /**
     * Stores velocity units that were created ad-hoc using {@link #combine(Unit, Time, String,
     * String)}. Does not store objects created directly by constructors.
     */
    @SuppressWarnings("rawtypes")
    private static final LongToObjectHashMap<Velocity> cache = new LongToObjectHashMap<>();
    private final D unit;
    private final Time period;

    Velocity(D unit, Time period, String name, String symbol) {
        super(unit.isBaseUnit() && period.isBaseUnit() ? null : combine(unit.baseUnit, period.baseUnit),
                unit.toBaseUnits(1) / period.toBaseUnits(1), name, symbol);
        this.unit = unit;
        this.period = period;
    }

    Velocity(Velocity<D> baseUnit, UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(baseUnit, toBaseConverter, fromBaseConverter, name, symbol);
        unit = baseUnit.unit;
        period = baseUnit.period;
    }

    /**
     * Generates a cache key used for cache lookups.
     */
    private static long cacheKey(Unit<?> numerator, Unit<?> denominator) {
        return ((long) numerator.hashCode()) << 32L | (denominator.hashCode() & 0xFFFFFFFFL);
    }

    /**
     * Creates a new velocity unit derived from an arbitrary numerator and time period units.
     *
     * <p>Results of this method are cached so future invocations with the same arguments will return
     * the pre-existing units instead of generating new identical ones.
     *
     * <pre>
     *   Velocity.combine(Kilograms, Second) // mass flow
     *   Velocity.combine(Feet, Millisecond) // linear speed
     *   Velocity.combine(Radians, Second) // angular speed
     *
     *   Velocity.combine(Feet.per(Second), Second) // linear acceleration in ft/s/s
     *   Velocity.combine(Radians.per(Second), Second) // angular acceleration
     * </pre>
     *
     * <p>It's recommended to use the convenience function {@link Unit#per(Time)} instead of calling
     * this factory directly.
     *
     * @param <D>       the type of the numerator unit
     * @param numerator the numerator unit
     * @param period    the period for unit time
     * @param name      the name of the new velocity unit
     * @param symbol    the symbol of the new velocity unit
     * @return the new unit
     */
    @NonNull
    @SuppressWarnings("unchecked")
    public static <D extends Unit<D>> Velocity<D> combine(@NonNull Unit<D> numerator, @NonNull Time period, @NonNull String name, @NonNull String symbol) {
        long key = cacheKey(numerator, period);
        if (cache.containsKey(key)) {
            return cache.get(key);
        }

        Velocity<D> velocity = new Velocity<>((D) numerator, period, name, symbol);
        cache.put(key, velocity);
        return velocity;
    }

    /**
     * Creates a new velocity unit derived from an arbitrary numerator and time period units.
     *
     * <p>Results of this method are cached so future invocations with the same arguments will return
     * the pre-existing units instead of generating new identical ones.
     *
     * <p>This method automatically generates a new name and symbol for the new velocity unit.
     *
     * <pre>
     *   Velocity.combine(Kilograms, Second) // mass flow
     *   Velocity.combine(Feet, Millisecond) // linear speed
     *   Velocity.combine(Radians, Second) // angular speed
     *
     *   Velocity.combine(Feet.per(Second), Second) // linear acceleration in ft/s/s
     *   Velocity.combine(Radians.per(Second), Second) // angular acceleration
     * </pre>
     *
     * <p>It's recommended to use the convenience function {@link Unit#per(Time)} instead of calling
     * this factory directly.
     *
     * @param <D>       the type of the numerator unit
     * @param numerator the numerator unit
     * @param period    the period for unit time
     * @return the new unit
     */
    @NonNull
    @SuppressWarnings("unchecked")
    public static <D extends Unit<D>> Velocity<D> combine(@NonNull Unit<D> numerator, @NonNull Time period) {
        long key = cacheKey(numerator, period);
        if (cache.containsKey(key)) {
            return cache.get(key);
        }

        String name = numerator.name() + " per " + period.name();
        String symbol = numerator.symbol() + "/" + period.symbol();

        Velocity<D> velocity = new Velocity<>((D) numerator, period, name, symbol);
        cache.put(key, velocity);
        return velocity;
    }

    /**
     * Gets the major unit being measured (e.g. Meters for Meters per Second).
     *
     * @return the major unit
     */
    public D getUnit() {
        return unit;
    }

    /**
     * Gets the period unit of the velocity, e.g. Seconds or Milliseconds.
     *
     * @return the period unit
     */
    @NonNull
    public Time getPeriod() {
        return period;
    }

    /**
     * Returns the reciprocal of this velocity.
     *
     * @return the reciprocal
     */
    @NonNull
    public Per<Time, D> reciprocal() {
        return period.per(unit);
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (other == null || getClass() != other.getClass()) {
            return false;
        }
        if (!super.equals(other)) {
            return false;
        }
        Velocity<?> velocity = (Velocity<?>) other;
        return unit.equals(velocity.unit) && period.equals(velocity.period);
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), unit, period);
    }
}
