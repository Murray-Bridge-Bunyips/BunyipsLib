package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecondPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.TurnConstraints;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
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
    public Double maxAngVelRadsPerSec;
    /**
     * Specified minimum angular acceleration in radians per second squared.
     */
    @Nullable
    public Double minAngAccelRadsPerSecSquared;
    /**
     * Specified maximum angular acceleration in radians per second squared.
     */
    @Nullable
    public Double maxAngAccelRadsPerSecSquared;

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
    public static Turn ofMaxAngVel(double maxAngVel, @NonNull Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), null, null);
    }

    /**
     * Create a new Turn object with the specified minimum angular acceleration.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMinAngAccel(double minAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(null, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), null);
    }

    /**
     * Create a new Turn object with the specified maximum angular acceleration.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public static Turn ofMaxAngAccel(double maxAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(null, null, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
    }

    /**
     * Compose this Turn object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity in radians per second.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxAngVel(double maxAngVel, @NonNull Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), minAngAccelRadsPerSecSquared, maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a minimum angular acceleration constraint.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMinAngAccel(double minAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a maximum angular acceleration constraint.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */
    @NonNull
    public Turn andMaxAngAccel(double maxAngAccel, @NonNull Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, minAngAccelRadsPerSecSquared, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
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
