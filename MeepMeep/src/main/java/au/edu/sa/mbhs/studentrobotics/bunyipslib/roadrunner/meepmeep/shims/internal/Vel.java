package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.RadiansPerSecond;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Velocity;


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

    public Double maxVelInchesPerSec;
    /**
     * Specified maximum angular velocity in radians per second.
     */

    public Double maxAngVelRadsPerSec;

    private Vel(Double maxVelInchesPerSec, Double maxAngVelRadsPerSec) {
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

    public static Vel ofMax(double maxVel, Velocity<Distance> unit) {
        return new Vel(unit.of(maxVel).in(InchesPerSecond), null);
    }

    /**
     * Create a new Vel object with the specified maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Vel object.
     */

    public static Vel ofMaxAng(double maxAngVel, Velocity<Angle> unit) {
        return new Vel(null, unit.of(maxAngVel).in(RadiansPerSecond));
    }

    /**
     * Compose this Vel object with a maximum translation velocity constraint.
     *
     * @param maxVel The maximum translation velocity.
     * @param unit   The unit of the maximum translation velocity.
     * @return The new Vel object.
     */

    public Vel andMax(double maxVel, Velocity<Distance> unit) {
        return new Vel(unit.of(maxVel).in(InchesPerSecond), maxAngVelRadsPerSec);
    }

    /**
     * Compose this Vel object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity.
     * @param unit      The unit of the maximum angular velocity.
     * @return The new Vel object.
     */

    public Vel andMaxAng(double maxAngVel, Velocity<Angle> unit) {
        return new Vel(maxVelInchesPerSec, unit.of(maxAngVel).in(RadiansPerSecond));
    }


    /**
     * Create a new Vel object with the specified maximum translation velocity.
     *
     * @param maxVel The maximum translation velocity.
     * @return The new Vel object.
     */

    public static Vel ofMax(Measure<Velocity<Distance>> maxVel) {
        return new Vel(maxVel.in(InchesPerSecond), null);
    }

    /**
     * Create a new Vel object with the specified maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity.
     * @return The new Vel object.
     */

    public static Vel ofMaxAng(Measure<Velocity<Angle>> maxAngVel) {
        return new Vel(null, maxAngVel.in(RadiansPerSecond));
    }

    /**
     * Compose this Vel object with a maximum translation velocity constraint.
     *
     * @param maxVel The maximum translation velocity.
     * @return The new Vel object.
     */

    public Vel andMax(Measure<Velocity<Distance>> maxVel) {
        return new Vel(maxVel.in(InchesPerSecond), maxAngVelRadsPerSec);
    }

    /**
     * Compose this Vel object with a maximum angular velocity constraint.
     *
     * @param maxAngVel The maximum angular velocity.
     * @return The new Vel object.
     */

    public Vel andMaxAng(Measure<Velocity<Angle>> maxAngVel) {
        return new Vel(maxVelInchesPerSec, maxAngVel.in(RadiansPerSecond));
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
