package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import androidx.annotation.NonNull;

/**
 * Drive coefficients that define gains for a Mecanum drivetrain.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class MecanumGains {
    /**
     * The gain for axial movement.
     */
    public double axialGain = 0.0;
    /**
     * The gain for lateral movement.
     */
    public double lateralGain = 0.0;
    /**
     * The gain for heading movement.
     */
    public double headingGain = 0.0;

    /**
     * The gain for axial velocity.
     */
    public double axialVelGain = 0.0;
    /**
     * The gain for lateral velocity.
     */
    public double lateralVelGain = 0.0;
    /**
     * The gain for heading velocity.
     */
    public double headingVelGain = 0.0;

    /**
     * Whether a path follower approach should be used instead of the default time-based trajectory.
     */
    public boolean pathFollowingEnabled;

    @NonNull
    @Override
    public String toString() {
        return "MecanumGains{" +
                "axialGain=" + axialGain +
                ", lateralGain=" + lateralGain +
                ", headingGain=" + headingGain +
                ", axialVelGain=" + axialVelGain +
                ", lateralVelGain=" + lateralVelGain +
                ", headingVelGain=" + headingVelGain +
                '}';
    }

    /**
     * Builder class to assist in creating a MecanumGains.
     */
    public static class Builder {
        private final MecanumGains gains = new MecanumGains();

        @NonNull
        public Builder setAxialGain(double axialGain) {
            gains.axialGain = axialGain;
            return this;
        }

        @NonNull
        public Builder setLateralGain(double lateralGain) {
            gains.lateralGain = lateralGain;
            return this;
        }

        @NonNull
        public Builder setHeadingGain(double headingGain) {
            gains.headingGain = headingGain;
            return this;
        }

        @NonNull
        public Builder setAxialVelGain(double axialVelGain) {
            gains.axialVelGain = axialVelGain;
            return this;
        }

        @NonNull
        public Builder setLateralVelGain(double lateralVelGain) {
            gains.lateralVelGain = lateralVelGain;
            return this;
        }

        @NonNull
        public Builder setHeadingVelGain(double headingVelGain) {
            gains.headingVelGain = headingVelGain;
            return this;
        }

        /**
         * Enabling this option will use an alternative path follower for trajectory tasks, rather
         * than a time-based trajectory. This is similar to a Pure Pursuit controller without lookahead.
         * <p>
         * More information on this mode can be found on the
         * RoadRunner <a href="https://rr.brott.dev/docs/v1-0/guides/path-following/">Wiki</a>.
         *
         * @param usePathFollowing whether to use a displacement-based path follower for trajectory tasks
         * @return this
         */
        @NonNull
        public Builder setPathFollowing(boolean usePathFollowing) {
            gains.pathFollowingEnabled = usePathFollowing;
            return this;
        }

        /**
         * Build the MecanumGains object.
         *
         * @return the MecanumGains object
         */
        @NonNull
        public MecanumGains build() {
            return gains;
        }
    }
}
