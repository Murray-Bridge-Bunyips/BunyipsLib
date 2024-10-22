package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

/**
 * Drive motion profile parameters for feedforward, pathing, and turn control.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class MotionProfile {
    /**
     * Static feedforward gain in tick units.
     */
    public double kS = 0;
    /**
     * Velocity feedforward gain in tick units.
     */
    public double kV = 0;
    /**
     * Acceleration feedforward gain in tick units.
     */
    public double kA = 0;

    /**
     * Maximum wheel velocity in inches per second.
     */
    public double maxWheelVel = 50;
    /**
     * Lower acceleration profile limit in inches per second squared.
     */
    public double minProfileAccel = -30;
    /**
     * Upper acceleration profile limit in inches per second squared.
     */
    public double maxProfileAccel = 30;

    /**
     * Maximum angular velocity in radians per second. Shared with the pathing profile.
     */
    public double maxAngVel = Math.PI;
    /**
     * Maximum angular acceleration in radians per second squared.
     */
    public double maxAngAccel = Math.PI;

    /**
     * Builder class to assist in creating a DriveProfile.
     */
    public static class Builder {
        private final MotionProfile profile = new MotionProfile();

        /**
         * Set the static feedforward gain in tick units.
         *
         * @param kS the static feedforward gain in tick units
         * @return the builder
         */
        @NonNull
        public Builder setKs(double kS) {
            profile.kS = kS;
            return this;
        }

        /**
         * Set the velocity feedforward gain in tick units.
         *
         * @param kV the velocity feedforward gain in tick units
         * @return the builder
         */
        @NonNull
        public Builder setKv(double kV) {
            profile.kV = kV;
            return this;
        }

        /**
         * Set the acceleration feedforward gain in tick units.
         *
         * @param kA the acceleration feedforward gain in tick units
         * @return the builder
         */
        @NonNull
        public Builder setKa(double kA) {
            profile.kA = kA;
            return this;
        }

        /**
         * Set the maximum wheel velocity in inches per second.
         *
         * @param maxWheelVel the maximum wheel velocity in inches per second
         * @return the builder
         */
        @NonNull
        public Builder setMaxWheelVel(double maxWheelVel) {
            profile.maxWheelVel = maxWheelVel;
            return this;
        }

        /**
         * Set the maximum wheel velocity.
         *
         * @param maxVelocity the maximum wheel velocity in desired units
         * @return the builder
         */
        @NonNull
        public Builder setMaxWheelVel(@NonNull Measure<Velocity<Distance>> maxVelocity) {
            profile.maxWheelVel = maxVelocity.in(InchesPerSecond);
            return this;
        }

        /**
         * Set the lower acceleration profile limit in inches per second squared.
         *
         * @param minProfileAccel the lower acceleration profile limit in inches per second squared
         * @return the builder
         */
        @NonNull
        public Builder setMinProfileAccel(double minProfileAccel) {
            profile.minProfileAccel = minProfileAccel;
            return this;
        }

        /**
         * Set the lower acceleration profile limit.
         *
         * @param minAccel the lower acceleration profile limit in desired units
         * @return the builder
         */
        @NonNull
        public Builder setMinProfileAccel(@NonNull Measure<Velocity<Velocity<Distance>>> minAccel) {
            profile.minProfileAccel = minAccel.in(InchesPerSecondPerSecond);
            return this;
        }

        /**
         * Set the upper acceleration profile limit in inches per second squared.
         *
         * @param maxProfileAccel the upper acceleration profile limit in inches per second squared
         * @return the builder
         */
        @NonNull
        public Builder setMaxProfileAccel(double maxProfileAccel) {
            profile.maxProfileAccel = maxProfileAccel;
            return this;
        }

        /**
         * Set the upper acceleration profile limit.
         *
         * @param maxAccel the upper acceleration profile limit in desired units
         * @return the builder
         */
        @NonNull
        public Builder setMaxProfileAccel(@NonNull Measure<Velocity<Velocity<Distance>>> maxAccel) {
            profile.maxProfileAccel = maxAccel.in(InchesPerSecondPerSecond);
            return this;
        }

        /**
         * Set the maximum angular velocity in radians per second.
         *
         * @param maxAngVel the maximum angular velocity in radians per second
         * @return the builder
         */
        @NonNull
        public Builder setMaxAngVel(double maxAngVel) {
            profile.maxAngVel = maxAngVel;
            return this;
        }

        /**
         * Set the maximum angular velocity.
         *
         * @param maxAngVel the maximum angular velocity in desired units
         * @return the builder
         */
        @NonNull
        public Builder setMaxAngVel(@NonNull Measure<Velocity<Angle>> maxAngVel) {
            profile.maxAngVel = maxAngVel.in(RadiansPerSecond);
            return this;
        }

        /**
         * Set the maximum angular acceleration in radians per second squared.
         *
         * @param maxAngAccel the maximum angular acceleration in radians per second squared
         * @return the builder
         */
        @NonNull
        public Builder setMaxAngAccel(double maxAngAccel) {
            profile.maxAngAccel = maxAngAccel;
            return this;
        }

        /**
         * Set the maximum angular acceleration.
         *
         * @param maxAngAccel the maximum angular acceleration in desired units
         * @return the builder
         */
        @NonNull
        public Builder setMaxAngAccel(@NonNull Measure<Velocity<Velocity<Angle>>> maxAngAccel) {
            profile.maxAngAccel = maxAngAccel.in(RadiansPerSecond.per(Second));
            return this;
        }

        /**
         * Build the drive profile.
         *
         * @return the drive profile
         */
        @NonNull
        public MotionProfile build() {
            return profile;
        }
    }
}
