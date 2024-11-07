/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import androidx.annotation.NonNull;

import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;

/**
 * A helper class that computes feedforward outputs for a simple permanent-magnet DC motor.
 * Units of this class are determined by the inputs to the gains.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/SimpleMotorFeedforward.java">Source</a>
 *
 * @since 3.5.0
 */
public class SimpleMotorFeedforward implements SystemController {
    /**
     * Static gain.
     */
    public double kS;
    /**
     * Velocity gain.
     */
    public double kV;
    /**
     * Acceleration gain.
     */
    public double kA;

    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier accelerationSupplier;

    /**
     * Creates a new SimpleMotorFeedforward with the specified gains and suppliers for the setpoints.
     *
     * @param kS           The static gain.
     * @param kV           The velocity gain.
     * @param kA           The acceleration gain.
     * @param velocity     The current velocity of the motor.
     * @param acceleration The current acceleration of the motor.
     */
    public SimpleMotorFeedforward(double kS, double kV, double kA, @NonNull DoubleSupplier velocity, @NonNull DoubleSupplier acceleration) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        velocitySupplier = velocity;
        accelerationSupplier = acceleration;
    }

    /**
     * Static gain.
     */
    public double getS() {
        return kS;
    }

    /**
     * Velocity gain.
     */
    public double getV() {
        return kV;
    }

    /**
     * Acceleration gain.
     */
    public double getA() {
        return kA;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @return The computed feedforward.
     */
    public double calculate() {
        double velocity = velocitySupplier.getAsDouble();
        return kS * Math.signum(velocity) + kV * velocity + kA * accelerationSupplier.getAsDouble();
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param current ignored for a feedforward calculation
     * @param target  ignored for a feedforward calculation
     * @return The computed feedforward.
     */
    @Override
    public double calculate(double current, double target) {
        return calculate();
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply
     * and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the motor.
     * @param acceleration The acceleration of the motor.
     * @return The maximum possible velocity at the given acceleration.
     */
    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - kS - acceleration * kA) / kV;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply
     * and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the motor.
     * @param acceleration The acceleration of the motor.
     * @return The minimum possible velocity at the given acceleration.
     */
    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + kS - acceleration * kA) / kV;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param velocity   The velocity of the motor.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - kS * Math.signum(velocity) - velocity * kV) / kA;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the motor.
     * @param velocity   The velocity of the motor.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kS, kV, kA};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 3) {
            throw new IllegalArgumentException("expected 3 coefficients, got " + coeffs.length);
        }
        kS = coeffs[0];
        kV = coeffs[1];
        kA = coeffs[2];
    }
}