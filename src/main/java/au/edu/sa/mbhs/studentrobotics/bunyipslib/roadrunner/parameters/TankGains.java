package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import org.apache.commons.math3.exception.NotStrictlyPositiveException;
import org.apache.commons.math3.exception.NumberIsTooLargeException;
import org.apache.commons.math3.exception.NumberIsTooSmallException;

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
        public Builder setRamseteZeta(double ramseteZeta) {
            if (ramseteZeta <= 0) {
                throw new NumberIsTooSmallException(ramseteZeta, 0, false);
            }
            if (ramseteZeta >= 1) {
                throw new NumberIsTooLargeException(ramseteZeta, 1, false);
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
        public Builder setRamseteBBar(double ramseteBBar) {
            if (ramseteBBar <= 0) {
                throw new NotStrictlyPositiveException(ramseteBBar);
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
        public Builder setTurnVelGain(double turnVelGain) {
            gains.turnVelGain = turnVelGain;
            return this;
        }

        /**
         * Build the gains.
         *
         * @return the TankGains object
         */
        public TankGains build() {
            return gains;
        }
    }
}
