package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.RadiansPerSecondPerSecond;

import com.acmerobotics.roadrunner.TurnConstraints;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Velocity;

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

    public Double maxAngVelRadsPerSec;
    /**
     * Specified minimum angular acceleration in radians per second squared.
     */

    public Double minAngAccelRadsPerSecSquared;
    /**
     * Specified maximum angular acceleration in radians per second squared.
     */

    public Double maxAngAccelRadsPerSecSquared;

    private Turn(Double maxAngVelRadsPerSec, Double minAngAccelRadsPerSecSquared, Double maxAngAccelRadsPerSecSquared) {
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

    public static Turn ofMaxVel(double maxAngVel, Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), null, null);
    }

    /**
     * Create a new Turn object with the specified minimum angular acceleration.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */

    public static Turn ofMinAccel(double minAngAccel, Velocity<Velocity<Angle>> unit) {
        return new Turn(null, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), null);
    }

    /**
     * Create a new Turn object with the specified maximum angular acceleration.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */

    public static Turn ofMaxAccel(double maxAngAccel, Velocity<Velocity<Angle>> unit) {
        return new Turn(null, null, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
    }

    /**
     * Compose this Turn object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity in radians per second.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Turn object.
     */

    public Turn andMaxVel(double maxAngVel, Velocity<Angle> unit) {
        return new Turn(unit.of(maxAngVel).in(RadiansPerSecond), minAngAccelRadsPerSecSquared, maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a minimum angular acceleration constraint.
     *
     * @param minAngAccel The minimum angular acceleration in radians per second squared.
     * @param unit        The unit of the minimum angular acceleration.
     * @return The new Turn object.
     */

    public Turn andMinAccel(double minAngAccel, Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, unit.of(minAngAccel).in(RadiansPerSecondPerSecond), maxAngAccelRadsPerSecSquared);
    }

    /**
     * Compose this Turn object with a maximum angular acceleration constraint.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared.
     * @param unit        The unit of the maximum angular acceleration.
     * @return The new Turn object.
     */

    public Turn andMaxAccel(double maxAngAccel, Velocity<Velocity<Angle>> unit) {
        return new Turn(maxAngVelRadsPerSec, minAngAccelRadsPerSecSquared, unit.of(maxAngAccel).in(RadiansPerSecondPerSecond));
    }

    /**
     * Get a built TurnConstraints object with the specified constraints.
     *
     * @param defaultConstraints The default constraints to use if no constraints are specified.
     * @return The built TurnConstraints object.
     */

    public TurnConstraints getOrDefault(TurnConstraints defaultConstraints) {
        return new TurnConstraints(
                maxAngVelRadsPerSec == null ? defaultConstraints.maxAngVel : maxAngVelRadsPerSec,
                minAngAccelRadsPerSecSquared == null ? defaultConstraints.minAngAccel : minAngAccelRadsPerSecSquared,
                maxAngAccelRadsPerSecSquared == null ? defaultConstraints.maxAngAccel : maxAngAccelRadsPerSecSquared
        );
    }
}
