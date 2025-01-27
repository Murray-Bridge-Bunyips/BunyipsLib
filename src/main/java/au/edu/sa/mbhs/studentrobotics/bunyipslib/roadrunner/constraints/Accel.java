package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecondPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

/**
 * Utility constructor for creating simple RoadRunner acceleration constraints.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class Accel {
    /**
     * Specified minimum acceleration in inches per second squared.
     */
    @Nullable
    public final Double minAccelInchesPerSecSquared;
    /**
     * Specified maximum acceleration in inches per second squared.
     */
    @Nullable
    public final Double maxAccelInchesPerSecSquared;

    private Accel(@Nullable Double minAccelInchesPerSecSquared, @Nullable Double maxAccelInchesPerSecSquared) {
        this.minAccelInchesPerSecSquared = minAccelInchesPerSecSquared;
        this.maxAccelInchesPerSecSquared = maxAccelInchesPerSecSquared;
    }

    /**
     * Create a new Accel object with the specified minimum acceleration.
     *
     * @param minAccel The minimum acceleration constraint.
     * @param unit     The unit of the minimum acceleration.
     * @return The new Accel object.
     */
    @NonNull
    public static Accel ofMin(double minAccel, @NonNull Velocity<Velocity<Distance>> unit) {
        return new Accel(unit.of(minAccel).in(InchesPerSecondPerSecond), null);
    }

    /**
     * Create a new Accel object with the specified maximum acceleration.
     *
     * @param maxAccel The maximum acceleration constraint.
     * @param unit     The unit of the maximum acceleration.
     * @return The new Accel object.
     */
    @NonNull
    public static Accel ofMax(double maxAccel, @NonNull Velocity<Velocity<Distance>> unit) {
        return new Accel(null, unit.of(maxAccel).in(InchesPerSecondPerSecond));
    }

    /**
     * Create a new Accel object with the specified minimum acceleration.
     *
     * @param minAccel The minimum acceleration constraint.
     * @return The new Accel object.
     */
    @NonNull
    public static Accel ofMin(@NonNull Measure<Velocity<Velocity<Distance>>> minAccel) {
        return new Accel(minAccel.in(InchesPerSecondPerSecond), null);
    }

    /**
     * Create a new Accel object with the specified maximum acceleration.
     *
     * @param maxAccel The maximum acceleration constraint.
     * @return The new Accel object.
     */
    @NonNull
    public static Accel ofMax(@NonNull Measure<Velocity<Velocity<Distance>>> maxAccel) {
        return new Accel(null, maxAccel.in(InchesPerSecondPerSecond));
    }

    /**
     * Compose this Accel object with a maximum acceleration constraint.
     *
     * @param maxAccel The maximum acceleration constraint.
     * @param unit     The unit of the maximum acceleration.
     * @return The new Accel object.
     */
    @NonNull
    public Accel andMax(double maxAccel, @NonNull Velocity<Velocity<Distance>> unit) {
        return new Accel(minAccelInchesPerSecSquared, unit.of(maxAccel).in(InchesPerSecondPerSecond));
    }

    /**
     * Compose this Accel object with a minimum acceleration constraint.
     *
     * @param minAccel The minimum acceleration constraint.
     * @param unit     The unit of the minimum acceleration.
     * @return The new Accel object.
     */
    @NonNull
    public Accel andMin(double minAccel, @NonNull Velocity<Velocity<Distance>> unit) {
        return new Accel(unit.of(minAccel).in(InchesPerSecondPerSecond), maxAccelInchesPerSecSquared);
    }

    /**
     * Compose this Accel object with a maximum acceleration constraint.
     *
     * @param maxAccel The maximum acceleration constraint.
     * @return The new Accel object.
     */
    @NonNull
    public Accel andMax(@NonNull Measure<Velocity<Velocity<Distance>>> maxAccel) {
        return new Accel(minAccelInchesPerSecSquared, maxAccel.in(InchesPerSecondPerSecond));
    }

    /**
     * Compose this Accel object with a minimum acceleration constraint.
     *
     * @param minAccel The minimum acceleration constraint.
     * @return The new Accel object.
     */
    @NonNull
    public Accel andMin(@NonNull Measure<Velocity<Velocity<Distance>>> minAccel) {
        return new Accel(minAccel.in(InchesPerSecondPerSecond), maxAccelInchesPerSecSquared);
    }

    /**
     * Get a built ProfileAccelConstraint object with the specified constraints.
     *
     * @param defaultConstraints The default constraints to use if no constraints are specified.
     * @return The built ProfileAccelConstraint object.
     */
    @NonNull
    public ProfileAccelConstraint getOrDefault(@NonNull ProfileAccelConstraint defaultConstraints) {
        return new ProfileAccelConstraint(
                minAccelInchesPerSecSquared == null ? defaultConstraints.minAccel : minAccelInchesPerSecSquared,
                maxAccelInchesPerSecSquared == null ? defaultConstraints.maxAccel : maxAccelInchesPerSecSquared
        );
    }
}
