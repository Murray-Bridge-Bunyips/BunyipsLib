package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.InchesPerSecondPerSecond;

import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Velocity;

/**
 * Utility constructor for creating simple RoadRunner acceleration constraints.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@SuppressWarnings("UnknownNullness")
public final class Accel {
    /**
     * Specified minimum acceleration in inches per second squared.
     */
    public Double minAccelInchesPerSecSquared;
    /**
     * Specified maximum acceleration in inches per second squared.
     */
    public Double maxAccelInchesPerSecSquared;

    private Accel(Double minAccelInchesPerSecSquared, Double maxAccelInchesPerSecSquared) {
        this.minAccelInchesPerSecSquared = minAccelInchesPerSecSquared;
        this.maxAccelInchesPerSecSquared = maxAccelInchesPerSecSquared;
    }

    /**
     * Create a new Accel object with the specified minimum acceleration.
     *
     * @param minAccel The minimum acceleration in inches per second squared.
     * @param unit     The unit of the minimum acceleration.
     * @return The new Accel object.
     */
    public static Accel ofMinAccel(double minAccel, Velocity<Velocity<Distance>> unit) {
        return new Accel(unit.of(minAccel).in(InchesPerSecondPerSecond), null);
    }

    /**
     * Create a new Accel object with the specified maximum acceleration.
     *
     * @param maxAccel The maximum acceleration in inches per second squared.
     * @param unit     The unit of the maximum acceleration.
     * @return The new Accel object.
     */
    public static Accel ofMaxAccel(double maxAccel, Velocity<Velocity<Distance>> unit) {
        return new Accel(null, unit.of(maxAccel).in(InchesPerSecondPerSecond));
    }

    /**
     * Compose this Accel object with a maximum acceleration constraint.
     *
     * @param maxAccel The maximum acceleration in inches per second squared.
     * @param unit     The unit of the maximum acceleration.
     * @return The new Accel object.
     */
    public Accel andMaxAccel(double maxAccel, Velocity<Velocity<Distance>> unit) {
        return new Accel(minAccelInchesPerSecSquared, unit.of(maxAccel).in(InchesPerSecondPerSecond));
    }

    /**
     * Compose this Accel object with a minimum acceleration constraint.
     *
     * @param minAccel The minimum acceleration in inches per second squared.
     * @param unit     The unit of the minimum acceleration.
     * @return The new Accel object.
     */
    public Accel andMinAccel(double minAccel, Velocity<Velocity<Distance>> unit) {
        return new Accel(unit.of(minAccel).in(InchesPerSecondPerSecond), maxAccelInchesPerSecSquared);
    }

    /**
     * Get a built ProfileAccelConstraint object with the specified constraints.
     *
     * @param defaultConstraints The default constraints to use if no constraints are specified.
     * @return The built ProfileAccelConstraint object.
     */
    public ProfileAccelConstraint getOrDefault(ProfileAccelConstraint defaultConstraints) {
        return new ProfileAccelConstraint(
                minAccelInchesPerSecSquared == null ? defaultConstraints.minAccel : minAccelInchesPerSecSquared,
                maxAccelInchesPerSecSquared == null ? defaultConstraints.maxAccel : maxAccelInchesPerSecSquared
        );
    }
}
