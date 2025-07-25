package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecondPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.TurnConstraints;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

/**
 * Utility constructor for creating RoadRunner turn constraints.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class Turn {
    /**
     * Specified maximum angular velocity in radians per second.
     */
    @Nullable
    public final Double maxAngVelRadsPerSec;
    /**
     * Specified minimum angular acceleration in radians per second squared.
     */
    @Nullable
    public final Double minAngAccelRadsPerSecSquared;
    /**
     * Specified maximum angular acceleration in radians per second squared.
     */
    @Nullable
    public final Double maxAngAccelRadsPerSecSquared;

    private Turn(@Nullable Double maxAngVelRadsPerSec, @Nullable Double minAngAccelRadsPerSecSquared, @Nullable Double maxAngAccelRadsPerSecSquared) {
        this.maxAngVelRadsPerSec = maxAngVelRadsPerSec;
        this.minAngAccelRadsPerSecSquared = minAngAccelRadsPerSecSquared;
        this.maxAngAccelRadsPerSecSquared = maxAngAccelRadsPerSecSquared;
    }

    /**
     * Create a new Turn object with the specified maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity in radians per second.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMaxVel(double maxAngVel, @NonNull Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), null, null);
    }

    /**
     * Create a new Turn object with the specified maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMaxVel(@NonNull Measure<Velocity<Angle>> maxAngVel) {
        return new Turn(maxAngVel.in(RadiansPerSecond), null, null);
    }

    /**
     * Create a new Turn object with the specified minimum angular acceleration.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMinAccel(double minAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(null, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), null);
    }

    /**
     * Create a new Turn object with the specified minimum angular acceleration.
     *
     * @param minAngAccel The minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMinAccel(@NonNull Measure<Velocity<Velocity<Angle>>> minAngAccel) {
        return new Turn(null, minAngAccel.in(RadiansPerSecondPerSecond), null);
    }

    /**
     * Create a new Turn object with the specified maximum angular acceleration.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMaxAccel(double maxAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(null, null, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
    }

    /**
     * Create a new Turn object with the specified maximum angular acceleration.
     *
     * @param maxAngAccel The maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMaxAccel(@NonNull Measure<Velocity<Velocity<Angle>>> maxAngAccel) {
        return new Turn(null, null, maxAngAccel.in(RadiansPerSecondPerSecond));
    }

    /**
     * Compose this Turn object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity in radians per second.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxVel(double maxAngVel, @NonNull Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), minAngAccelRadsPerSecSquared, maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxVel(@NonNull Measure<Velocity<Angle>> maxAngVel) {
        return new Turn(maxAngVel.in(RadiansPerSecond), minAngAccelRadsPerSecSquared, maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a minimum angular acceleration constraint.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMinAccel(double minAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a minimum angular acceleration constraint.
     *
     * @param minAngAccel The minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMinAccel(@NonNull Measure<Velocity<Velocity<Angle>>> minAngAccel) {
        return new Turn(maxAngVelRadsPerSec, minAngAccel.in(RadiansPerSecondPerSecond), maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a maximum angular acceleration constraint.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxAccel(double maxAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, minAngAccelRadsPerSecSquared, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
    }

    /**
     * Compose this Turn object with a maximum angular acceleration constraint.
     *
     * @param maxAngAccel The maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxAccel(@NonNull Measure<Velocity<Velocity<Angle>>> maxAngAccel) {
        return new Turn(maxAngVelRadsPerSec, minAngAccelRadsPerSecSquared, maxAngAccel.in(RadiansPerSecondPerSecond));
    }

    /**
     * Get a built TurnConstraints object with the specified constraints.
     *
     * @param defaultConstraints The default constraints to use if no constraints are specified.
     * @return The built TurnConstraints object.
     */
    @NonNull
    public TurnConstraints getOrDefault(@NonNull TurnConstraints defaultConstraints) {
        return new TurnConstraints(
                maxAngVelRadsPerSec == null ? defaultConstraints.maxAngVel : maxAngVelRadsPerSec,
                minAngAccelRadsPerSecSquared == null ? defaultConstraints.minAngAccel : minAngAccelRadsPerSecSquared,
                maxAngAccelRadsPerSecSquared == null ? defaultConstraints.maxAngAccel : maxAngAccelRadsPerSecSquared
        );
    }
}
