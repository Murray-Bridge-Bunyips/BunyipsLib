package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.constraints;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

/**
 * Utility constructor for creating simple RoadRunner translation velocity constraints.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public final class Vel {
    /**
     * Specified maximum translation velocity in inches per second.
     */
    @Nullable
    public Double maxVelInchesPerSec;
    /**
     * Specified maximum angular velocity in radians per second.
     */
    @Nullable
    public Double maxAngVelRadsPerSec;

    private Vel(@Nullable Double maxVelInchesPerSec, @Nullable Double maxAngVelRadsPerSec) {
        this.maxVelInchesPerSec = maxVelInchesPerSec;
        this.maxAngVelRadsPerSec = maxAngVelRadsPerSec;
    }

    /**
     * Create a new Vel object with the specified maximum translation velocity.
     *
     * @param maxVel The maximum translation velocity.
     * @param unit   The unit of the maximum translation velocity.
     * @return The new Vel object.
     */
    public static Vel ofMaxVel(double maxVel, Velocity<Distance> unit) {
        return new Vel(unit.of(maxVel).in(InchesPerSecond), null);
    }

    /**
     * Create a new Vel object with the specified maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Vel object.
     */
    public static Vel ofMaxAngVel(double maxAngVel, Velocity<Angle> unit) {
        return new Vel(null, unit.of(maxAngVel).in(RadiansPerSecond));
    }

    /**
     * Compose this Vel object with a maximum translation velocity constraint.
     *
     * @param maxVel The maximum translation velocity.
     * @param unit   The unit of the maximum translation velocity.
     * @return The new Vel object.
     */
    public Vel andMaxVel(double maxVel, Velocity<Distance> unit) {
        return new Vel(unit.of(maxVel).in(InchesPerSecond), maxAngVelRadsPerSec);
    }

    /**
     * Compose this Vel object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Vel object.
     */
    public Vel andMaxAngVel(double maxAngVel, Velocity<Angle> unit) {
        return new Vel(maxVelInchesPerSec, unit.of(maxAngVel).in(RadiansPerSecond));
    }

    /**
     * Get a built MinVelConstraint object with the specified constraints.
     *
     * @param defaultVelConstraint    The default translation velocity constraint to use if no constraint is specified.
     * @param defaultAngVelConstraint The default angular velocity constraint to use if no constraint is specified.
     * @return The built MinVelConstraint object.
     */
    public MinVelConstraint getOrDefault(VelConstraint defaultVelConstraint, VelConstraint defaultAngVelConstraint) {
        return new MinVelConstraint(Arrays.asList(
                maxVelInchesPerSec == null ? defaultVelConstraint : new TranslationalVelConstraint(maxVelInchesPerSec),
                maxAngVelRadsPerSec == null ? defaultAngVelConstraint : new AngularVelConstraint(maxAngVelRadsPerSec)
        ));
    }
}
