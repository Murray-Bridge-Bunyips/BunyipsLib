/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile.
 * Users should call {@code reset()} when they first start running the controller to avoid unwanted behavior.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ProfiledPIDController.java">Source</a>
 *
 * @since 3.5.0
 */
public class ProfiledPIDController extends PIDController {
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints constraints;

    /**
     * Allocates a ProfiledPIDController with the given constants for kP, kI, and kD.
     *
     * @param kP          The proportional coefficient.
     * @param kI          The integral coefficient.
     * @param kD          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDController(double kP, double kI, double kD,
                                 @NonNull TrapezoidProfile.Constraints constraints) {
        super(kP, kI, kD);
        this.constraints = constraints;
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
     * Not to be confused with {@link #getSetpoint()}.
     *
     * @return The current setpoint.
     */
    @NonNull
    public TrapezoidProfile.State getSetPoint() {
        return setpoint;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next output of the PID controller.
     */
    @Override
    public double calculate(double measurement) {
        if (isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (maxContinuousInput - minContinuousInput) / 2.0;
            double goalMinDistance = Mathf.wrap(goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance = Mathf.wrap(setpoint.position - measurement, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            goal.position = goalMinDistance + measurement;
            setpoint.position = setpointMinDistance + measurement;
        }

        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(getPeriod());
        setSetpoint(setpoint.position);
        return super.calculate(measurement);
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
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     * @return this
     */
    @NonNull
    public ProfiledPIDController reset(@NonNull TrapezoidProfile.State measurement) {
        reset();
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
}