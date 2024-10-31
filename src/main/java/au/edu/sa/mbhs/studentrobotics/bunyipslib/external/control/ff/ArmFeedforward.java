/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Volts;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Voltage;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor
 * acting against the force of gravity on a beam suspended at an angle), adjusted to use WPIUnits for unit assertion.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ArmFeedforward.java">Source</a>
 *
 * @since 3.5.0
 */
public class ArmFeedforward implements SystemController {
    private final Supplier<Measure<Angle>> position;
    private final Supplier<Measure<Velocity<Angle>>> velocity;
    private final Supplier<Measure<Velocity<Velocity<Angle>>>> acceleration;
    private double kS;
    private double kCos;
    private double kV;
    private double kA;

    /**
     * Creates a new ArmFeedforward with the specified gains and suppliers for the setpoints.
     *
     * @param kS           The static gain.
     * @param kCos         The gravity gain.
     * @param kV           The velocity gain.
     * @param kA           The acceleration gain.
     * @param position     The current angular position of the arm.
     * @param velocity     The current angular velocity of the arm.
     * @param acceleration The current angular acceleration of the arm.
     */
    public ArmFeedforward(double kS, double kCos, double kV, double kA,
                          @NonNull Supplier<Measure<Angle>> position, @NonNull Supplier<Measure<Velocity<Angle>>> velocity,
                          @NonNull Supplier<Measure<Velocity<Velocity<Angle>>>> acceleration) {
        this.kS = kS;
        this.kCos = kCos;
        this.kV = kV;
        this.kA = kA;

        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    /**
     * Static gain.
     */
    public double getS() {
        return kS;
    }

    /**
     * Gravity gain.
     */
    public double getCos() {
        return kCos;
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
        double positionRadians = position.get().in(Radians);
        double velocityRadPerSec = velocity.get().in(RadiansPerSecond);
        double accelRadPerSecSquared = acceleration.get().in(RadiansPerSecondPerSecond);
        return kS * Math.signum(velocityRadPerSec) + kCos * Math.cos(positionRadians)
                + kV * velocityRadPerSec
                + kA * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param current ignored for a feedforward calculation
     * @param target  ignored for a feedforward calculation
     * @return controller output
     */
    @Override
    public double calculate(double current, double target) {
        return calculate();
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    @NonNull
    public Measure<Velocity<Angle>> maxAchievableVelocity(@NonNull Measure<Voltage> maxVoltage, @NonNull Measure<Angle> angle, @NonNull Measure<Velocity<Velocity<Angle>>> acceleration) {
        // Assume max velocity is positive
        return RadiansPerSecond.of((maxVoltage.in(Volts) - kS - Math.cos(angle.in(Radians)) * kCos - acceleration.in(RadiansPerSecondPerSecond) * kA) / kV);
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    @NonNull
    public Measure<Velocity<Angle>> minAchievableVelocity(@NonNull Measure<Voltage> maxVoltage, @NonNull Measure<Angle> angle, @NonNull Measure<Velocity<Velocity<Angle>>> acceleration) {
        // Assume min velocity is negative, ks flips sign
        return RadiansPerSecond.of((-maxVoltage.in(Volts) + kS - Math.cos(angle.in(Radians)) * kCos - acceleration.in(RadiansPerSecondPerSecond) * kA) / kV);
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm.
     * @param velocity   The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    @NonNull
    public Measure<Velocity<Velocity<Angle>>> maxAchievableAcceleration(@NonNull Measure<Voltage> maxVoltage, @NonNull Measure<Angle> angle, @NonNull Measure<Velocity<Angle>> velocity) {
        double v = velocity.in(RadiansPerSecond);
        return RadiansPerSecondPerSecond.of((maxVoltage.in(Volts) - kS * Math.signum(v) - Math.cos(angle.in(Radians)) * kCos - v * kV) / kA);
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm.
     * @param velocity   The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    @NonNull
    public Measure<Velocity<Velocity<Angle>>> minAchievableAcceleration(@NonNull Measure<Voltage> maxVoltage, @NonNull Measure<Angle> angle, @NonNull Measure<Velocity<Angle>> velocity) {
        return maxAchievableAcceleration(maxVoltage.negate(), angle, velocity);
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kS, kCos, kV, kA};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 4) {
            throw new IllegalArgumentException("expected 4 coefficients, got " + coeffs.length);
        }
        kS = coeffs[0];
        kCos = coeffs[1];
        kV = coeffs[2];
        kA = coeffs[3];
    }
}