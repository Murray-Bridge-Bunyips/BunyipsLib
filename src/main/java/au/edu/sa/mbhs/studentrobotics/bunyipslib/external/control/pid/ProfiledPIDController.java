/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid;

import androidx.annotation.NonNull;

import java.util.Optional;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ProfiledPIDController.java">Source</a>
 *
 * @since 3.5.0
 */
public class ProfiledPIDController implements SystemController {
    /**
     * The underlying PID controller used for ProfiledPID.
     */
    public final PIDController controller;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints constraints;

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDController(double Kp, double Ki, double Kd,
                                 @NonNull TrapezoidProfile.Constraints constraints) {
        controller = new PIDController(Kp, Ki, Kd);
        this.constraints = constraints;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Differential coefficient
     * @return this
     */
    @NonNull
    public ProfiledPIDController setPID(double Kp, double Ki, double Kd) {
        controller.setPID(Kp, Ki, Kd);
        return this;
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return controller.getP();
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp proportional coefficient
     * @return this
     */
    @NonNull
    public ProfiledPIDController setP(double Kp) {
        controller.setP(Kp);
        return this;
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return controller.getI();
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki integral coefficient
     * @return this
     */
    @NonNull
    public ProfiledPIDController setI(double Ki) {
        controller.setI(Ki);
        return this;
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return controller.getD();
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd differential coefficient
     * @return this
     */
    @NonNull
    public ProfiledPIDController setD(double Kd) {
        controller.setD(Kd);
        return this;
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return controller.getPeriod();
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    @NonNull
    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     * @return this
     */
    @NonNull
    public ProfiledPIDController setGoal(@NonNull TrapezoidProfile.State goal) {
        this.goal = goal;
        return this;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     * @return this
     */
    @NonNull
    public ProfiledPIDController setGoal(double goal) {
        this.goal = new TrapezoidProfile.State(goal, 0);
        return this;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the tolerance.
     */
    public boolean atGoal() {
        return atSetpoint() && goal.equals(setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     * @return this
     */
    @NonNull
    public ProfiledPIDController setConstraints(@NonNull TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        return this;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    @NonNull
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the tolerance.
     */
    public boolean atSetpoint() {
        return controller.atSetPoint();
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetpoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @return this
     */
    @NonNull
    public ProfiledPIDController setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
        return this;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetpoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     * @return this
     */
    @NonNull
    public ProfiledPIDController setTolerance(double positionTolerance, double velocityTolerance) {
        controller.setTolerance(positionTolerance, velocityTolerance);
        return this;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return controller.getPositionError();
    }

    /**
     * Returns the change in error per second.
     */
    public double getVelocityError() {
        return controller.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement) {
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(getPeriod());
        return controller.calculate(measurement, setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param newGoal     The new goal of the controller.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement, @NonNull TrapezoidProfile.State newGoal) {
        goal = newGoal;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param newGoal     The new goal of the controller.
     * @return The next output of the PID controller.
     */
    @Override
    public double calculate(double measurement, double newGoal) {
        setGoal(newGoal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement    The current measurement of the process variable.
     * @param newGoal        The new goal of the controller.
     * @param newConstraints Velocity and acceleration constraints for goal.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement, @NonNull TrapezoidProfile.State newGoal,
                            @NonNull TrapezoidProfile.Constraints newConstraints) {
        constraints = newConstraints;
        return calculate(measurement, newGoal);
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    @Override
    public void reset() {
        controller.reset();
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     * @return this
     */
    @NonNull
    public ProfiledPIDController reset(@NonNull TrapezoidProfile.State measurement) {
        controller.reset();
        setpoint = measurement;
        return this;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     * @return this
     */
    @NonNull
    public ProfiledPIDController reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
        return this;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is
     *                         assumed to be zero.
     * @return this
     */
    @NonNull
    public ProfiledPIDController reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
        return this;
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{getP(), getI(), getD()};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        controller.setCoefficients(coeffs);
    }

    @NonNull
    @Override
    public Optional<PIDFController> pidf() {
        // Note: The controller as returned here will be limited in trapezoidal limitation
        return Optional.of(controller);
    }
}