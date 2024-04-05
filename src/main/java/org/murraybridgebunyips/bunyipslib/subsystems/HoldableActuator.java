package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.DoubleSupplier;

/**
 * Controls a generic holdable arm, that may be actuated by a user's input
 * but will hold its position when the input is released.
 *
 * @see Rotator
 * @author Lucas Bubner, 2024
 */
public class HoldableActuator extends BunyipsSubsystem {
    // Power to hold the actuator in place
    private double HOLDING_POWER = 1.0;
    // Power to move the actuator when in auto mode
    private double MOVING_POWER = 0.7;
    private DcMotorEx motor;
    private double power;
    private boolean lockout;
    private String name = "Actuator";

    /**
     * @param motor the motor to control as the actuator
     */
    public HoldableActuator(DcMotorEx motor) {
        if (!assertParamsNotNull(motor)) return;
        this.motor = motor;
        if (motor == null) return;
        // Assumes current arm position is the zero position
        // TODO: Limit switch variant of HoldableActuator
        // We could also opt to use the EncoderMotor class to manage some of this state, but it's not necessary
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set the name of the actuator to display in telemetry.
     * @param newName the name to set
     * @return this
     */
    public HoldableActuator withName(String newName) {
        name = newName;
        return this;
    }

    /**
     * Set the holding power of the actuator.
     * @param targetPower the power to set
     * @return this
     */
    public HoldableActuator withHoldingPower(double targetPower) {
        HOLDING_POWER = targetPower;
        return this;
    }

    /**
     * Set the moving power of the actuator.
     * @param targetPower the power to set
     * @return this
     */
    public HoldableActuator withMovingPower(double targetPower) {
        MOVING_POWER = targetPower;
        return this;
    }

    /**
     * Move the actuator with a supplier of power. Should be a default task.
     *
     * @param powerSupplier the power value supplier
     * @return a task to move the actuator
     */
    public Task controlTask(DoubleSupplier powerSupplier) {
        return new ContinuousTask(() -> setPower(powerSupplier.getAsDouble()), this, false).withName("JoystickControlTask");
    }

    public HoldableActuator setPower(double p) {
        power = Range.clip(p, -1.0, 1.0);
        return this;
    }

    /**
     * Set the power of the actuator.
     *
     * @param p the power to set
     * @return a task to set the power
     */
    public Task setPowerTask(double p) {
        return new RunTask(() -> setPower(p), this, false).withName("SetPowerTask");
    }

    /**
     * Run the actuator for a certain amount of time.
     *
     * @param p    the power to run at
     * @param time the time to run for in seconds
     * @return a task to run the actuator
     */
    public Task runForTask(double p, double time) {
        return new Task(time, this, true) {
            @Override
            public void init() {
            }

            @Override
            public void periodic() {
                power = p;
            }

            @Override
            public void onFinish() {
                power = 0;
            }

            @Override
            public boolean isTaskFinished() {
                return false;
            }
        }.withName("RunForTask");
    }

    // TODO: Task for running the actuator until it can detect it is homed

    /**
     * Set the position of the actuator.
     * <p></p>
     * Informally known as the Doinky-Rubber-Bandy Task
     *
     * @param targetPosition the position to set
     * @return a task to set the position
     */
    public Task gotoTask(int targetPosition) {
        return new NoTimeoutTask(this, true) {
            @Override
            public void init() {
                lockout = true;
            }

            @Override
            public void periodic() {
                motor.setTargetPosition(targetPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(MOVING_POWER);
            }

            @Override
            public void onFinish() {
                lockout = false;
            }

            @Override
            public boolean isTaskFinished() {
                return !motor.isBusy();
            }
        }.withName("RunToPositionTask");
    }

    /**
     * Delta the position of the actuator.
     *
     * @param deltaPosition the delta to add to the current position of the actuator
     * @return a task to delta the position
     */
    public Task deltaTask(int deltaPosition) {
        return gotoTask(motor.getCurrentPosition() + deltaPosition);
    }

    @Override
    protected void periodic() {
        if (lockout) {
            opMode.addTelemetry("%: MOVING to % ticks", name, motor.getCurrentPosition());
            return;
        }

        if (power == 0.0) {
            // Hold arm in place
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(HOLDING_POWER);
        } else {
            // Move arm in accordance with the user's input
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(power);
        }

        opMode.addTelemetry("%: % at % ticks", name, power == 0.0 ? "HOLDING" : "MOVING", motor.getCurrentPosition());
    }
}
