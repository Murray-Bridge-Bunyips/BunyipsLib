package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner;

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
     * Builder class to assist in creating a MecanumGains.
     */
    public static class Builder {
        private final MecanumGains gains = new MecanumGains();

        public Builder setAxialGain(double axialGain) {
            gains.axialGain = axialGain;
            return this;
        }

        public Builder setLateralGain(double lateralGain) {
            gains.lateralGain = lateralGain;
            return this;
        }

        public Builder setHeadingGain(double headingGain) {
            gains.headingGain = headingGain;
            return this;
        }

        public Builder setAxialVelGain(double axialVelGain) {
            gains.axialVelGain = axialVelGain;
            return this;
        }

        public Builder setLateralVelGain(double lateralVelGain) {
            gains.lateralVelGain = lateralVelGain;
            return this;
        }

        public Builder setHeadingVelGain(double headingVelGain) {
            gains.headingVelGain = headingVelGain;
            return this;
        }

        /**
         * Build the MecanumGains object.
         *
         * @return the MecanumGains object
         */
        public MecanumGains build() {
            return gains;
        }
    }
}
