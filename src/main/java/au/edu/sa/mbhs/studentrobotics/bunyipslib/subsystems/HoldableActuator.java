package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Amps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Encoder;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Current;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ContinuousTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.RunTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Controls a generic holdable motor, that may be actuated by a user's input
 * but will hold its position when the input is released.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class HoldableActuator extends BunyipsSubsystem {
    /**
     * Tasks for HoldableActuator.
     */
    public final Tasks tasks = new Tasks();

    private final HashMap<TouchSensor, Integer> switchMapping = new HashMap<>();
    // Power to hold the actuator in place
    private double HOLDING_POWER = 1.0;
    // Power to move the actuator when in auto mode
    private double MOVING_POWER = 0.7;
    // Number of greater than zero velocity hits required for Home Task
    private int ZERO_HIT_THRESHOLD = 30;
    // Overcurrent for Home Task
    private Measure<Current> OVERCURRENT = Amps.of(4);
    // Time of which the Home Task Overcurrent needs to be exceeded
    private Measure<Time> OVERCURRENT_TIME = Seconds.of(1.0);
    // Maximum time spent in the Home Task before it is assumed completed
    private Measure<Time> HOMING_TIMEOUT = Seconds.of(5);
    // Encoder lower limit in ticks
    private long MIN_LIMIT = -Long.MAX_VALUE;
    // Encoder upper limit in ticks
    private long MAX_LIMIT = Long.MAX_VALUE;
    // Lower power clamp
    private double LOWER_POWER = -1.0;
    // Upper power clamp
    private double UPPER_POWER = 1.0;
    // Tolerance for the actuator in ticks, default 2
    private int TOLERANCE = 2;
    private DcMotorEx motor;
    private Encoder encoder;
    private TouchSensor topSwitch;
    private TouchSensor bottomSwitch;
    private int homingDirection = -1;
    private boolean zeroed;
    private boolean userLatch;
    private double userPower;
    private double motorPower;
    // 5.1.0, user controls can now opt to adjust the setpoint instead of the power
    // but this functionality is disabled by default for consistency
    private boolean userControlsSetpoint;
    private Mode inputMode = Mode.USER_POWER;
    private DoubleSupplier setpointDeltaMultiplier = () -> 1;

    /**
     * Create a new HoldableActuator.
     *
     * @param motor the motor to control
     */
    public HoldableActuator(@NonNull DcMotor motor) {
        if (!assertParamsNotNull(motor)) return;
        this.motor = (DcMotorEx) motor;
        // Always default to BRAKE because HoldableActuators are meant to hold
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Assumes current arm position is the zero position, the user may home manually if required using setInitTask
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPower(HOLDING_POWER);
        // Encoder instance is used for awareness of the encoder position, we generally don't care about direction
        // as it is handled by the motor object itself, unless we're using a Motor object then we can conveniently
        // hook into it
        if (motor instanceof Motor) {
            encoder = ((Motor) motor).getEncoder();
        } else {
            encoder = new Encoder(this.motor::getCurrentPosition, this.motor::getVelocity);
        }
    }

    /**
     * Set the target tolerance of the actuator.
     *
     * @param tolerance    the tolerance to set in encoder ticks
     * @param applyToMotor whether to apply this tolerance to the motor as well as the task checks
     * @return this
     */
    @NonNull
    public HoldableActuator withTolerance(int tolerance, boolean applyToMotor) {
        if (applyToMotor)
            motor.setTargetPositionTolerance(tolerance);
        TOLERANCE = tolerance;
        return this;
    }

    /**
     * Set the zero hit threshold to how many greater than or equal to zero velocity hits are required for the Home Task.
     * If the actuator has a continuous negative velocity of zero for this many hits, the Home Task will complete.
     *
     * @param threshold the new threshold of continuous hits of zero velocity to complete homing. Default is 30.
     * @return this
     * @see #disableHomingZeroHits()
     */
    @NonNull
    public HoldableActuator withHomingZeroHits(int threshold) {
        ZERO_HIT_THRESHOLD = threshold;
        return this;
    }

    /**
     * Disable the zero hit threshold for the Home Task.
     *
     * @return this
     */
    @NonNull
    public HoldableActuator disableHomingZeroHits() {
        return withHomingZeroHits(0);
    }

    /**
     * Set the overcurrent threshold for the Home Task.
     * If this current is reached during a Home Task, for the set duration set here, the home task will end.
     *
     * @param current the current which if exceeded in a Home Task will finish the reset. Default is 4A.
     * @param forTime the time the current must be exceeded for to finish the reset. Useful for filtering out momentary spikes. Default is 1s.
     * @return this
     * @see #disableHomingOvercurrent()
     */
    @NonNull
    public HoldableActuator withHomingOvercurrent(@NonNull Measure<Current> current, @NonNull Measure<Time> forTime) {
        OVERCURRENT = current;
        OVERCURRENT_TIME = forTime;
        return this;
    }

    /**
     * Disable the overcurrent threshold for the Home Task.
     *
     * @return this
     */
    @NonNull
    public HoldableActuator disableHomingOvercurrent() {
        return withHomingOvercurrent(Amps.of(0), Seconds.of(0));
    }

    /**
     * Set the timeout for the Home Task.
     *
     * @param timeout the time to set for the Home Task to complete. Default is 5s.
     * @return this
     */
    @NonNull
    public HoldableActuator withHomingTimeout(@NonNull Measure<Time> timeout) {
        HOMING_TIMEOUT = timeout;
        return this;
    }

    /**
     * Disable the timeout for the Home Task.
     *
     * @return this
     */
    @NonNull
    public HoldableActuator disableHomingTimeout() {
        return withHomingTimeout(Task.INFINITE_TIMEOUT);
    }

    /**
     * Set the top limit switch of the actuator to use in encoder awareness.
     *
     * @param topLimitSwitch the limit switch to set as the top switch where the arm would be at the max position
     * @return this
     */
    @NonNull
    public HoldableActuator withTopSwitch(@NonNull TouchSensor topLimitSwitch) {
        topSwitch = topLimitSwitch;
        return this;
    }

    /**
     * Set the bottom limit switch of the actuator to use in homing and encoder awareness.
     *
     * @param bottomLimitSwitch the limit switch to set as the bottom switch where the arm would be "homed"
     * @return this
     * @see #disableHomingZeroHits()
     * @see #disableHomingOvercurrent()
     */
    @NonNull
    public HoldableActuator withBottomSwitch(@NonNull TouchSensor bottomLimitSwitch) {
        bottomSwitch = bottomLimitSwitch;
        return this;
    }

    /**
     * Map a limit switch to a position on the actuator, which will update the current encoder reading
     * to the position of the switch when pressed.
     * <p>
     * This is different from the bottom switch, and top switches, which are used as software stops for the actuator.
     * This method operates to reduce encoder drift, as if a switch is pressed by the actuator, the position can be mapped.
     * It may be wise to map the top switch here if used, which will increase encoder accuracy.
     * The bottom limit switch does not need to be mapped since it is used to reset the encoder, therefore
     * setting the reading to 0.
     *
     * @param switchSensor the switch sensor to map
     * @param position     the position to map the switch to in encoder ticks of the actuator
     * @return this
     */
    @NonNull
    public HoldableActuator map(@NonNull TouchSensor switchSensor, int position) {
        switchMapping.put(switchSensor, position);
        return this;
    }

    /**
     * Get the mapped position of a limit switch.
     *
     * @param switchSensor the switch sensor to get the mapped position of, must be mapped
     * @return the mapped position of the switch sensor, nullable if not mapped
     */
    @Nullable
    public Integer getMappedPosition(@NonNull TouchSensor switchSensor) {
        return switchMapping.get(switchSensor);
    }

    /**
     * Set the holding power of the actuator. Note: this power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set
     * @return this
     */
    @NonNull
    public HoldableActuator withHoldingPower(double targetPower) {
        HOLDING_POWER = targetPower;
        return this;
    }

    /**
     * Set the moving power of the actuator, where a positive value will bring the arm upwards (away from bottom).
     * Note: this power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set
     * @return this
     */
    @NonNull
    public HoldableActuator withMovingPower(double targetPower) {
        MOVING_POWER = targetPower;
        return this;
    }

    /**
     * Set the lower power clamp of the actuator.
     *
     * @param lowerPower the lower power clamp to set
     * @return this
     */
    @NonNull
    public HoldableActuator withLowerPowerClamp(double lowerPower) {
        return withPowerClamps(lowerPower, UPPER_POWER);
    }

    /**
     * Set the upper power clamp of the actuator.
     *
     * @param upperPower the upper power clamp to set
     * @return this
     */
    @NonNull
    public HoldableActuator withUpperPowerClamp(double upperPower) {
        return withPowerClamps(LOWER_POWER, upperPower);
    }

    /**
     * Set the lower and upper power clamps of the actuator.
     *
     * @param lowerPower the lower power clamp to set
     * @param upperPower the upper power clamp to set
     * @return this
     */
    @NonNull
    public HoldableActuator withPowerClamps(double lowerPower, double upperPower) {
        LOWER_POWER = Mathf.clamp(lowerPower, -1, 1);
        UPPER_POWER = Mathf.clamp(upperPower, -1, 1);
        return this;
    }

    /**
     * Set the lower limit of the actuator.
     *
     * @param minLimit the lower limit to set in encoder ticks
     * @return this
     */
    @NonNull
    public HoldableActuator withLowerLimit(long minLimit) {
        MIN_LIMIT = minLimit;
        return this;
    }

    /**
     * Set the upper limit of the actuator.
     *
     * @param maxLimit the upper limit to set in encoder ticks
     * @return this
     */
    @NonNull
    public HoldableActuator withUpperLimit(long maxLimit) {
        MAX_LIMIT = maxLimit;
        return this;
    }

    /**
     * Set the encoder limits of the actuator.
     *
     * @param minLimit the lower limit of the actuator in encoder ticks
     * @param maxLimit the upper limit of the actuator in encoder ticks
     * @return this
     */
    @NonNull
    public HoldableActuator withEncoderLimits(long minLimit, long maxLimit) {
        MIN_LIMIT = minLimit;
        MAX_LIMIT = maxLimit;
        return this;
    }

    /**
     * Calling this method will enable the user input mode to instead adjust the setpoint dynamically, rather
     * than unlocking the setpoint then relocking it when the mode transitions to holding.
     * <p>
     * This mode is useful to call on high-frequency system controllers (like those accomplished via {@link Motor},
     * and switches over the manual control from raw input to system controls.
     *
     * @param setpointDeltaMultiplier the multiplicative scale to translate power into target position delta, which is a supplier
     *                                to allow for any functional patterns (e.g. deltaTime). Default of {@code () -> 1}.
     * @return this
     * @see #disableHomingOvercurrent()
     */
    @NonNull
    public HoldableActuator enableUserSetpointControl(@NonNull DoubleSupplier setpointDeltaMultiplier) {
        this.setpointDeltaMultiplier = setpointDeltaMultiplier;
        userControlsSetpoint = true;
        if (inputMode == Mode.USER_POWER)
            inputMode = Mode.USER_SETPOINT;
        return this;
    }

    /**
     * Calling this method will restore the user input functionality translating into direct power on the motor.
     *
     * @return this
     * @see #enableUserSetpointControl(DoubleSupplier)
     */
    @NonNull
    public HoldableActuator disableUserSetpointControl() {
        userControlsSetpoint = false;
        return this;
    }

    /**
     * Instantaneously set the user input power for the actuator.
     *
     * @param p power level in domain [-1.0, 1.0], will be clamped
     * @return this
     */
    @NonNull
    public HoldableActuator setPower(double p) {
        userPower = Mathf.clamp(p, LOWER_POWER, UPPER_POWER);
        return this;
    }

    @Override
    protected void periodic() {
        double current = encoder.getPosition();
        double target = motor.getTargetPosition();
        double newTarget = -1;

        switch (inputMode) {
            case AUTO:
                // Paranoia safety guard to ensure the motor does not enter RUN_TO_POSITION mode without a target
                try {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } catch (Exception e) {
                    // Cannot catch ActionNotSupportedException due to package protections, so we will handle a generic exception
                    // In the event this does fail, we will fallback to the default mode.
                    setInputModeToUser();
                    break;
                }
                motorPower = MOVING_POWER * Math.signum(target - current);
                opMode(o -> o.telemetry.add("%: <font color='#FF5F1F'>MOVING -> %/% ticks</font> [%tps]", this, current, target, Math.round(encoder.getVelocity())));
                break;
            case HOMING:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorPower = MOVING_POWER * Math.signum(homingDirection);
                opMode(o -> o.telemetry.add("%: <font color='yellow'><b>HOMING</b></font> [%tps]", this, Math.round(encoder.getVelocity())));
                break;
            case USER_POWER:
                if (userPower == 0.0) {
                    // Hold arm in place
                    if (!userLatch) {
                        newTarget = current;
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        userLatch = true;
                    }
                    motorPower = HOLDING_POWER * Math.signum(target - current);
                } else {
                    userLatch = false;
                    // Move arm in accordance with the user's input
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorPower = userPower;
                }
                opMode(o -> o.telemetry.add("%: % at % ticks [%tps]", this, userPower == 0.0 ? "<font color='green'>HOLDING</font>" : "<font color='#FF5F1F'><b>MOVING</b></font>", current, Math.round(encoder.getVelocity())));
                break;
            case USER_SETPOINT:
                newTarget = target + userPower * setpointDeltaMultiplier.getAsDouble();
                boolean systemResponse = motor.isBusy();
                motorPower = (systemResponse ? MOVING_POWER : HOLDING_POWER) * Math.signum(target - current);
                opMode(o -> o.telemetry.add("%: % at % ticks, % error [%tps]", this, !systemResponse ? "<font color='green'>SUSTAINING</font>" : "<font color='#FF5F1F'><b>RESPONDING</b></font>", current, target - current, Math.round(encoder.getVelocity())));
                break;
        }

        for (TouchSensor limitSwitch : switchMapping.keySet()) {
            // Map encoder to limit switch positions
            if (limitSwitch.isPressed()) {
                Integer ticks = switchMapping.get(limitSwitch);
                if (ticks != null)
                    encoder.setKnownPosition(ticks);
            }
        }

        if (bottomSwitch != null) {
            if (bottomSwitch.isPressed() && motorPower < 0) {
                // Cancel and stop any tasks that would move the actuator out of bounds as defined by the limit switch
                setInputModeToUser();
                motorPower = 0;
                if (target < 0)
                    newTarget = 0;
            }

            if (bottomSwitch.isPressed() && !zeroed) {
                DcMotor.RunMode prev = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // Must propagate now as we're switching the mode
                motor.setTargetPosition(0);
                motor.setMode(prev);
                // Ensure we only run the reset once every time the switch is pressed
                zeroed = true;
            }

            if (!bottomSwitch.isPressed())
                zeroed = false;
        }

        if (topSwitch != null && topSwitch.isPressed() && motorPower > 0) {
            setInputModeToUser();
            motorPower = 0;
            if (target > current)
                newTarget = current;
        }

        if (topSwitch != null && bottomSwitch != null && topSwitch.isPressed() && bottomSwitch.isPressed()) {
            Dbg.warn(getClass(), "%Warning: Both limit switches were pressed at the same time. This indicates an impossible system state.", isDefaultName() ? "" : "(" + name + ") ");
        }

        if (inputMode != Mode.HOMING && ((current < MIN_LIMIT && motorPower < 0.0) || (current > MAX_LIMIT && motorPower > 0.0))) {
            // Cancel any tasks that would move the actuator out of bounds by autonomous operation
            setInputModeToUser();
            motorPower = 0.0;
        }

        if (newTarget != -1) {
            motor.setTargetPosition((int) Math.round(newTarget));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(Mathf.clamp(motorPower, LOWER_POWER, UPPER_POWER));
    }

    @Override
    protected void onDisable() {
        motor.setPower(0);
    }

    private void setInputModeToUser() {
        userLatch = false;
        inputMode = userControlsSetpoint ? Mode.USER_SETPOINT : Mode.USER_POWER;
    }

    private enum Mode {
        AUTO,
        HOMING,
        USER_POWER,
        USER_SETPOINT
    }

    /**
     * Tasks for HoldableActuator, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Move the actuator with a supplier of power. Should be a default task.
         *
         * @param powerSupplier the power value supplier
         * @return a task to move the actuator
         */
        @NonNull
        public Task control(@NonNull DoubleSupplier powerSupplier) {
            return new ContinuousTask(() -> HoldableActuator.this.setPower(powerSupplier.getAsDouble()))
                    .onSubsystem(HoldableActuator.this, false)
                    .withName("Supplier Control");
        }

        /**
         * Set the power of the actuator.
         *
         * @param p the power to set
         * @return a task to set the power
         */
        @NonNull
        public Task setPower(double p) {
            return new RunTask(() -> HoldableActuator.this.setPower(p))
                    .onSubsystem(HoldableActuator.this, false)
                    .withName("Set Power");
        }

        /**
         * Run the actuator for a certain amount of time.
         *
         * @param time the time to run for
         * @param pwr  the power to run at
         * @return a task to run the actuator
         */
        @NonNull
        public Task runFor(@NonNull Measure<Time> time, double pwr) {
            return new Task(time) {
                @Override
                public void init() {
                    setInputModeToUser();
                }

                @Override
                public void periodic() {
                    // Will hijack the user power by constantly setting it
                    userPower = pwr;
                }

                @Override
                public void onFinish() {
                    userPower = 0;
                }

                @Override
                public boolean isTaskFinished() {
                    return false;
                }
            }.onSubsystem(HoldableActuator.this, true).withName("Run For Time");
        }

        /**
         * Home the actuator based on encoders against a hard stop or limit switch. This task ignores
         * the lower and upper limits as defined by this class.
         *
         * @return a task to home the actuator
         */
        @NonNull
        public Task home() {
            return new Task(HOMING_TIMEOUT) {
                private ElapsedTime overcurrentTimer;
                private double previousAmpAlert;
                private double zeroHits;

                @Override
                protected void init() {
                    previousAmpAlert = motor.getCurrentAlert(CurrentUnit.AMPS);
                    // Stop now if the switch is already pressed
                    if (bottomSwitch != null && bottomSwitch.isPressed()) {
                        finishNow();
                        return;
                    }
                    motor.setCurrentAlert(OVERCURRENT.in(Amps), CurrentUnit.AMPS);
                    zeroHits = 0;
                    homingDirection = -1;
                    inputMode = Mode.HOMING;
                }

                @Override
                protected void periodic() {
                    if (ZERO_HIT_THRESHOLD <= 0) return;
                    if (encoder.getVelocity() >= 0) {
                        zeroHits++;
                    } else {
                        zeroHits = 0;
                    }
                }

                @Override
                protected void onFinish() {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setCurrentAlert(previousAmpAlert, CurrentUnit.AMPS);
                    setInputModeToUser();
                }

                @Override
                protected boolean isTaskFinished() {
                    boolean bottomedOut = bottomSwitch != null && bottomSwitch.isPressed();
                    boolean velocityZeroed = ZERO_HIT_THRESHOLD > 0 && zeroHits >= ZERO_HIT_THRESHOLD;
                    boolean overCurrent = OVERCURRENT.magnitude() > 0 && motor.isOverCurrent();
                    if (OVERCURRENT_TIME.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                        overcurrentTimer = new ElapsedTime();
                    } else if (!overCurrent) {
                        overcurrentTimer = null;
                    }
                    boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= OVERCURRENT_TIME.in(Seconds);
                    return inputMode != Mode.HOMING || (bottomedOut || velocityZeroed || sustainedOvercurrent);
                }
            }.onSubsystem(HoldableActuator.this, true).withName("Return To Home");
        }

        /**
         * Send the actuator to the top limit, using the constants for homing. This effectively is a
         * home task, but instead runs in the opposite direction.
         * <p>
         * Note: Without an interrupt or top limit switch, this may be a dangerous task to call. Use with care, as
         * this task is intended for homing operations in the opposite direction instead of simply travelling
         * to some arbitrary limit while respecting the upper and lower bounds.
         *
         * @return a task to ceiling the actuator
         */
        @NonNull
        public Task ceil() {
            return new Task(HOMING_TIMEOUT) {
                private ElapsedTime overcurrentTimer;
                private double previousAmpAlert;
                private double zeroHits;

                @Override
                protected void init() {
                    previousAmpAlert = motor.getCurrentAlert(CurrentUnit.AMPS);
                    // Stop now if the switch is already pressed
                    if (topSwitch != null && topSwitch.isPressed()) {
                        finishNow();
                        return;
                    }
                    motor.setCurrentAlert(OVERCURRENT.in(Amps), CurrentUnit.AMPS);
                    zeroHits = 0;
                    homingDirection = 1;
                    inputMode = Mode.HOMING;
                }

                @Override
                protected void periodic() {
                    if (ZERO_HIT_THRESHOLD <= 0) return;
                    if (encoder.getVelocity() <= 0) {
                        zeroHits++;
                    } else {
                        zeroHits = 0;
                    }
                }

                @Override
                protected void onFinish() {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setCurrentAlert(previousAmpAlert, CurrentUnit.AMPS);
                    setInputModeToUser();
                }

                @Override
                protected boolean isTaskFinished() {
                    boolean toppedOut = topSwitch != null && topSwitch.isPressed();
                    boolean velocityZeroed = ZERO_HIT_THRESHOLD > 0 && zeroHits >= ZERO_HIT_THRESHOLD;
                    boolean overCurrent = OVERCURRENT.magnitude() > 0 && motor.isOverCurrent();
                    if (OVERCURRENT_TIME.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                        overcurrentTimer = new ElapsedTime();
                    } else if (!overCurrent) {
                        overcurrentTimer = null;
                    }
                    boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= OVERCURRENT_TIME.in(Seconds);
                    return inputMode != Mode.HOMING || (toppedOut || velocityZeroed || sustainedOvercurrent);
                }
            }.onSubsystem(HoldableActuator.this, true).withName("Travel To Ceiling");
        }

        /**
         * Set the position of the actuator based on a mapped limit switch.
         * This task will run the actuator to the position mapped by the limit switch, and has a condition to
         * stop the actuator if the limit switch is pressed.
         *
         * @param limitSwitch the position to set based on the limit switch, MUST be mapped by the actuator
         * @return a task to set the position
         */
        @NonNull
        public Task goTo(@NonNull TouchSensor limitSwitch) {
            Integer position = switchMapping.get(limitSwitch);
            if (position == null) {
                Dbg.warn(getClass(), "%Attempted to go to a limit switch that was not mapped. This task will not run.", isDefaultName() ? "" : "(" + name + ") ");
                return new RunTask();
            }
            // Since this is a static mapping we can return the task
            return goTo(position).until(limitSwitch::isPressed).withName("Run To Limit Switch");
        }

        /**
         * Set the position of the actuator.
         * <p></p>
         * Informally known as the Doinky-Rubber-Bandy Task
         *
         * @param targetPosition the position to set
         * @return a task to set the position
         */
        @NonNull
        public Task goTo(int targetPosition) {
            return new Task() {
                @Override
                public void init() {
                    motor.setTargetPosition(targetPosition);
                    // Motor power is controlled in the periodic method
                    motor.setPower(0);
                    inputMode = Mode.AUTO;
                }

                @Override
                public void periodic() {
                    // no-op
                }

                @Override
                public void onFinish() {
                    setInputModeToUser();
                }

                @Override
                public boolean isTaskFinished() {
                    return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(targetPosition, encoder.getPosition(), TOLERANCE));
                }
            }.onSubsystem(HoldableActuator.this, true).withName("Run To Position");
        }

        /**
         * Delta the position of the actuator.
         *
         * @param deltaPosition the delta to add to the current position of the actuator
         * @return a task to delta the position
         */
        @NonNull
        public Task delta(int deltaPosition) {
            // Must redefine the task since this is a deferred task
            return new Task() {
                private int target;

                @Override
                public void init() {
                    target = encoder.getPosition() + deltaPosition;
                    motor.setTargetPosition(target);
                    // Motor power is controlled in the periodic method
                    motor.setPower(0);
                    inputMode = Mode.AUTO;
                }

                @Override
                public void periodic() {
                    // no-op
                }

                @Override
                public void onFinish() {
                    setInputModeToUser();
                }

                @Override
                public boolean isTaskFinished() {
                    return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(target, encoder.getPosition(), TOLERANCE));
                }
            }.onSubsystem(HoldableActuator.this, true).withName("Run To Delta");
        }
    }
}
