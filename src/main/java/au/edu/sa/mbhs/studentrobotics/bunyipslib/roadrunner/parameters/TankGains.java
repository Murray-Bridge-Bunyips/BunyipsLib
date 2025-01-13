package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import androidx.annotation.NonNull;

/**
 * Drive coefficients that define gains for a Tank drivetrain.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class TankGains {
    /**
     * RAMSETE zeta gain. Must be in the range (0, 1).
     */
    public double ramseteZeta = 0.7;
    /**
     * RAMSETE beta bar gain. Must be greater than 0.
     */
    public double ramseteBBar = 2.0;

    /**
     * The gain for turning movement.
     */
    public double turnGain = 0.0;
    /**
     * The gain for turning velocity.
     */
    public double turnVelGain = 0.0;

    /**
     * Copy the parameters of this tank gains to another.
     *
     * @param other the other tank gains
     */
    public void copyTo(@NonNull TankGains other) {
        other.ramseteZeta = ramseteZeta;
        other.ramseteBBar = ramseteBBar;
        other.turnGain = turnGain;
        other.turnVelGain = turnVelGain;
    }

    @NonNull
    @Override
    public String toString() {
        return "TankGains{" +
                "ramseteZeta=" + ramseteZeta +
                ", ramseteBBar=" + ramseteBBar +
                ", turnGain=" + turnGain +
                ", turnVelGain=" + turnVelGain +
                '}';
    }

    /**
     * Builder class to assist in creating a TankGains.
     */
    public static class Builder {
        private final TankGains gains = new TankGains();

        /**
         * Set the RAMSETE zeta gain. Must be in the range (0, 1).
         *
         * @param ramseteZeta the RAMSETE zeta gain
         * @return the builder
         */
        @NonNull
        public Builder setRamseteZeta(double ramseteZeta) {
            if (ramseteZeta <= 0) {
                throw new IllegalArgumentException("zeta cannot be 0 or negative");
            }
            if (ramseteZeta >= 1) {
                throw new IllegalArgumentException("zeta cannot be greater than 1");
            }
            gains.ramseteZeta = ramseteZeta;
            return this;
        }

        /**
         * Set the RAMSETE beta bar gain. Must be greater than 0.
         *
         * @param ramseteBBar the RAMSETE beta bar gain
         * @return the builder
         */
        @NonNull
        public Builder setRamseteBBar(double ramseteBBar) {
            if (ramseteBBar <= 0) {
                throw new IllegalArgumentException("bBar cannot be zero or negative");
            }
            gains.ramseteBBar = ramseteBBar;
            return this;
        }

        /**
         * Set the gain for turning movement.
         *
         * @param turnGain the gain for turning movement as determined through tuning
         * @return the builder
         */
        @NonNull
        public Builder setTurnGain(double turnGain) {
            gains.turnGain = turnGain;
            return this;
        }

        /**
         * Set the gain for turning velocity.
         *
         * @param turnVelGain the gain for turning velocity as determined through tuning
         * @return the builder
         */
        @NonNull
        public Builder setTurnVelGain(double turnVelGain) {
            gains.turnVelGain = turnVelGain;
            return this;
        }

        /**
         * Build the gains.
         *
         * @return the TankGains object
         */
        @NonNull
        public TankGains build() {
            return gains;
        }
    }
}
