package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.DoubleSupplier;

/**
 * Controls a generic holdable arm, that may be actuated by a user's input
 * but will hold its position when the input is released.
 *
 * @author Lucas Bubner, 2024
 * @see Rotator
 */
public class HoldableActuator extends BunyipsSubsystem {
    // Power to hold the actuator in place
    private double HOLDING_POWER = 1.0;
    // Power to move the actuator when in auto mode
    private double MOVING_POWER = 0.7;
    // targetPosition tolerance in encoder ticks, default of 10
    private int TOLERANCE = 10;
    // Number of zero hits required for Home Task
    private int ZERO_HIT_THRESHOLD = 30;
    // Name of the actuator for telemetry
    private String NAME = "Actuator";
    private DcMotorEx motor;
    private TouchSensor bottomSwitch;
    private boolean zeroed;
    private double userPower;
    private double motorPower;
    private Mode inputMode = Mode.USER;

    /**
     * @param motor the motor to control as the actuator
     */
    public HoldableActuator(DcMotorEx motor) {
        if (!assertParamsNotNull(motor)) return;
        this.motor = motor;
        if (motor == null) return;
        // Assumes current arm position is the zero position, the user may home manually if required
        // We could also opt to use the EncoderMotor class to manage some of this state, but it's not necessary
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(HOLDING_POWER);
    }

    /**
     * Set the name of the actuator to display in telemetry.
     *
     * @param newName the name to set
     * @return this
     */
    public HoldableActuator withName(String newName) {
        NAME = newName;
        return this;
    }

    /**
     * Set the target tolerance of the actuator.
     *
     * @param tolerance the tolerance to set in encoder ticks
     * @param applyToMotor whether to apply this tolerance to the motor as well as the task checks
     * @return this
     */
    public HoldableActuator withTolerance(int tolerance, boolean applyToMotor) {
        if (applyToMotor)
            motor.setTargetPositionTolerance(tolerance);
        TOLERANCE = tolerance;
        return this;
    }

    /**
     * Set the zero hit threshold to how many zero velocity hits are required for the Home Task.
     * Set to 0 or less to have the Home Task rely solely on a limit switch provided by
     * {@link #withBottomSwitch(TouchSensor) withBottomSwitch}.
     *
     * @param threshold the new threshold of continuous hits of zero velocity to complete homing. Set to 0 or less to disable threshold.
     * @return this
     */
    public HoldableActuator withZeroHitThreshold(int threshold) {
        ZERO_HIT_THRESHOLD = threshold;
        return this;
    }

    /**
     * Set the bottom limit switch of the actuator to use in homing and encoder awareness.
     *
     * @param bottomLimitSwitch the limit switch to set as the bottom switch where the arm would be "homed"
     * @return this
     */
    public HoldableActuator withBottomSwitch(TouchSensor bottomLimitSwitch) {
        bottomSwitch = bottomLimitSwitch;
        return this;
    }

    /**
     * Set the holding power of the actuator.
     *
     * @param targetPower the power to set
     * @return this
     */
    public HoldableActuator withHoldingPower(double targetPower) {
        HOLDING_POWER = targetPower;
        return this;
    }

    /**
     * Set the moving power of the actuator, where a positive value will bring the arm upwards (away from bottom).
     *
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
        userPower = Range.clip(p, -1.0, 1.0);
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
     * @param time the time to run for
     * @return a task to run the actuator
     */
    public Task runForTask(double p, Measure<Time> time) {
        return new Task(time, this, true) {
            @Override
            public void init() {
                inputMode = Mode.USER;
            }

            @Override
            public void periodic() {
                // Will hijack the user power by constantly setting it
                userPower = p;
            }

            @Override
            public void onFinish() {
                userPower = 0;
            }

            @Override
            public boolean isTaskFinished() {
                return false;
            }
        }.withName("RunForTask");
    }

    /**
     * Home the actuator based on encoder tick velocity against a hard stop or limit switch.
     *
     * @return a task to home the actuator
     */
    // TODO: Test
    public Task homeTask() {
        return new NoTimeoutTask() {
            double zeroHits;

            @Override
            protected void init() {
                zeroHits = 0;
                inputMode = Mode.HOMING;
            }

            @Override
            protected void periodic() {
                if (ZERO_HIT_THRESHOLD <= 0) return;
                if (motor.getVelocity() >= 0) {
                    zeroHits++;
                } else {
                    zeroHits = 0;
                }
            }

            @Override
            protected void onFinish() {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                inputMode = Mode.USER;
            }

            @Override
            protected boolean isTaskFinished() {
                boolean bottomedOut = bottomSwitch != null && bottomSwitch.isPressed();
                boolean velocityZeroed = ZERO_HIT_THRESHOLD > 0 && zeroHits >= ZERO_HIT_THRESHOLD;
                return bottomedOut || velocityZeroed;
            }
        }.withName("HomeTask");
    }

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
                motor.setTargetPosition(targetPosition);
                inputMode = Mode.AUTO;
            }

            @Override
            public void periodic() {
                // no-op
            }

            @Override
            public void onFinish() {
                inputMode = Mode.USER;
            }

            @Override
            public boolean isTaskFinished() {
                return !motor.isBusy() && Math.abs(targetPosition - motor.getCurrentPosition()) < TOLERANCE;
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
        return new NoTimeoutTask(this, true) {
            private int target;

            @Override
            public void init() {
                target = motor.getCurrentPosition() + deltaPosition;
                motor.setTargetPosition(target);
                inputMode = Mode.AUTO;
            }

            @Override
            public void periodic() {
                // no-op
            }

            @Override
            public void onFinish() {
                inputMode = Mode.USER;
            }

            @Override
            public boolean isTaskFinished() {
                return !motor.isBusy() && Math.abs(target - motor.getCurrentPosition()) < TOLERANCE;
            }
        }.withName("DeltaPositionTask");
    }

    @Override
    protected void periodic() {
        switch (inputMode) {
            case AUTO:
                // Paranoia safety guard to ensure the motor does not enter RUN_TO_POSITION mode without a target
                try {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } catch (Exception e) {
                    // Cannot catch ActionNotSupportedException due to package protections, so we will handle a generic exception
                    // In the event this does fail, we will fallback to the default mode.
                    inputMode = Mode.USER;
                    break;
                }
                motorPower = MOVING_POWER;
                opMode.addTelemetry("%: MOVING to %/% ticks", NAME, motor.getTargetPosition(), motor.getCurrentPosition());
                break;
            case HOMING:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorPower = -MOVING_POWER;
                break;
            case USER:
                if (userPower == 0.0) {
                    // Hold arm in place
                    motor.setTargetPosition(motor.getCurrentPosition());
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorPower = HOLDING_POWER;
                } else {
                    // Move arm in accordance with the user's input
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorPower = userPower;
                }
                opMode.addTelemetry("%: % at % ticks", NAME, userPower == 0.0 ? "HOLDING" : "MOVING", motor.getCurrentPosition());
                break;
        }

        if (bottomSwitch != null) {
            if (bottomSwitch.isPressed() && !zeroed) {
                DcMotor.RunMode prev = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(0);
                motor.setMode(prev);
                // Ensure we only run the reset once every time the switch is pressed
                zeroed = true;
            }
            if (!bottomSwitch.isPressed())
                zeroed = false;
        }

        motor.setPower(motorPower);
    }

    private enum Mode {
        AUTO,
        HOMING,
        USER
    }
}
