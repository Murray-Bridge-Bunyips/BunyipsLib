package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Amps;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
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
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.ContinuousTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.RunTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Controls a generic encoder motor that can be controlled through various means and commanded to hold a position.
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
    private final ElapsedTime sustainedOvercurrent = new ElapsedTime();
    private final ElapsedTime sustainedTolerated = new ElapsedTime();
    private double rtpPower = 1.0;
    private double homePower = 0.7;
    private int zeroHitThreshold = 30;
    private Measure<Time> overcurrentTime;
    private Measure<Time> homingTimeout = Seconds.of(5);
    private Measure<Time> maxSteadyState;
    private long minLimit = -Long.MAX_VALUE;
    private long maxLimit = Long.MAX_VALUE;
    private double lowerPower = -1.0;
    private double upperPower = 1.0;
    private int tolerance = LynxConstants.DEFAULT_TARGET_POSITION_TOLERANCE;
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
    private Mode inputMode = Mode.USER_POWER;
    private UnaryFunction userSetpointControl;
    private double lastTime = -1;

    /**
     * Create a new HoldableActuator.
     *
     * @param motor the motor to control
     */
    public HoldableActuator(@Nullable DcMotor motor) {
        if (!assertParamsNotNull(motor)) return;
        assert motor != null;
        this.motor = (DcMotorEx) motor;
        // Always default to BRAKE because HoldableActuators are meant to hold
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setTargetPosition(this.motor.getCurrentPosition());
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPower(rtpPower);
        // Encoder instance is used for awareness of the encoder position, we generally don't care about direction
        // as it is handled by the motor object itself, unless we're using a Motor object then we can conveniently
        // hook into it
        if (motor instanceof Motor) {
            encoder = ((Motor) motor).encoder;
        } else {
            encoder = new Encoder(this.motor::getCurrentPosition, this.motor::getVelocity);
        }
        withOvercurrent(Amps.of(8), Seconds.of(2));
    }

    /**
     * Set the target tolerance of the actuator.
     *
     * @param tolerance    the tolerance to set in encoder ticks
     * @param applyToMotor whether to apply this tolerance to the motor as well as the task checks, default true
     * @return this
     */
    @NonNull
    public HoldableActuator withTolerance(int tolerance, boolean applyToMotor) {
        if (applyToMotor)
            motor.setTargetPositionTolerance(tolerance);
        this.tolerance = tolerance;
        return this;
    }

    /**
     * Set the target tolerance of the actuator.
     * Applies the tolerance to the motor object as well as the task checks.
     *
     * @param tolerance the tolerance to set in encoder ticks
     * @return this
     */
    @NonNull
    public HoldableActuator withTolerance(int tolerance) {
        return withTolerance(tolerance, true);
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
        zeroHitThreshold = threshold;
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
     * Set the overcurrent threshold used for stall detection.
     * <p>
     * If this current is reached during a state where stalling is monitored, for the set duration set here, the actuator
     * will auto-reset the motor.
     *
     * @param current the current which if exceeded will intervene. Default is 8A.
     * @param forTime the time the current must be exceeded for to execute the reset. Useful for filtering out momentary spikes. Default is 2s.
     * @return this
     * @see #disableOvercurrent()
     */
    @NonNull
    public HoldableActuator withOvercurrent(@NonNull Measure<Current> current, @NonNull Measure<Time> forTime) {
        motor.setCurrentAlert(current.in(Amps), CurrentUnit.AMPS);
        overcurrentTime = forTime;
        return this;
    }

    /**
     * Defines a max amount of time that the system can be responding for. If this time is elapsed, the target
     * position will be reset to the current position. This option is to avoid stall and endpoint conditions when
     * the target position cannot be achieved by the actuator.
     *
     * @param maxSteadyState the maximum amount of time a system response can last before any steady state error is zeroed
     * @return this
     */
    @NonNull
    public HoldableActuator withMaxSteadyStateTime(@Nullable Measure<Time> maxSteadyState) {
        this.maxSteadyState = maxSteadyState;
        return this;
    }

    /**
     * Disable the max steady state timer. Disabled by default.
     *
     * @return this
     */
    @NonNull
    public HoldableActuator disableMaxSteadyStateTime() {
        maxSteadyState = null;
        return this;
    }

    /**
     * Disable the overcurrent threshold for stall detection.
     *
     * @return this
     * @see #withOvercurrent(Measure, Measure)
     */
    @NonNull
    public HoldableActuator disableOvercurrent() {
        overcurrentTime = Seconds.zero();
        return this;
    }

    /**
     * Set the timeout for the Home Task.
     *
     * @param timeout the time to set for the Home Task to complete. Default is 5s.
     * @return this
     */
    @NonNull
    public HoldableActuator withHomingTimeout(@NonNull Measure<Time> timeout) {
        homingTimeout = timeout;
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
    public HoldableActuator withTopSwitch(@Nullable TouchSensor topLimitSwitch) {
        if (topLimitSwitch == null)
            Dbg.error(getClass(), "%Supplied top limit switch is null! Ignoring.", isDefaultName() ? "" : "(" + name + ") ");
        topSwitch = topLimitSwitch;
        return this;
    }

    /**
     * Set the bottom limit switch of the actuator to use in homing and encoder awareness.
     *
     * @param bottomLimitSwitch the limit switch to set as the bottom switch where the arm would be "homed"
     * @return this
     * @see #disableHomingZeroHits()
     * @see #disableOvercurrent()
     */
    @NonNull
    public HoldableActuator withBottomSwitch(@Nullable TouchSensor bottomLimitSwitch) {
        if (bottomLimitSwitch == null)
            Dbg.error(getClass(), "%Supplied bottom limit switch is null! Ignoring.", isDefaultName() ? "" : "(" + name + ") ");
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
    public HoldableActuator map(@Nullable TouchSensor switchSensor, int position) {
        if (switchSensor == null) return this;
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
    public Integer getMappedPosition(@Nullable TouchSensor switchSensor) {
        return switchMapping.get(switchSensor);
    }

    /**
     * Set the auto response power magnitude of the actuator, where a positive value will bring the arm upwards (away from bottom).
     * This value is used to scale automatic responses by the actuator. Default of 1.0.
     * <p>
     * Note: This power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set (magnitude)
     * @return this
     */
    @NonNull
    public HoldableActuator withAutoPower(double targetPower) {
        rtpPower = Math.abs(targetPower);
        return this;
    }

    /**
     * Set the homing power magnitude of the actuator, where a positive value will bring the arm upwards (away from bottom).
     * This power will be used as the raw power in a home or ceiling task. Default of 0.7.
     * <p>
     * Note: This power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set (magnitude)
     * @return this
     */
    @NonNull
    public HoldableActuator withHomingPower(double targetPower) {
        homePower = Math.abs(targetPower);
        return this;
    }

    /**
     * Set the lower power clamp of the actuator.
     *
     * @param lowerPower the lower power clamp to set (signed)
     * @return this
     */
    @NonNull
    public HoldableActuator withLowerPowerClamp(double lowerPower) {
        return withPowerClamps(lowerPower, upperPower);
    }

    /**
     * Set the upper power clamp of the actuator.
     *
     * @param upperPower the upper power clamp to set (signed)
     * @return this
     */
    @NonNull
    public HoldableActuator withUpperPowerClamp(double upperPower) {
        return withPowerClamps(lowerPower, upperPower);
    }

    /**
     * Set the lower and upper power clamps of the actuator.
     *
     * @param lowerPower the lower power clamp to set (signed)
     * @param upperPower the upper power clamp to set (signed)
     * @return this
     */
    @NonNull
    public HoldableActuator withPowerClamps(double lowerPower, double upperPower) {
        this.lowerPower = Mathf.clamp(lowerPower, -1, 1);
        this.upperPower = Mathf.clamp(upperPower, -1, 1);
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
        this.minLimit = minLimit;
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
        this.maxLimit = maxLimit;
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
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        return this;
    }

    /**
     * Calling this method will enable the user input mode to instead adjust the setpoint dynamically, rather
     * than unlocking the setpoint then relocking it when the mode transitions to holding.
     * <p>
     * This mode is useful to call on high-frequency system controllers (like those accomplished via {@link Motor},
     * and switches over the manual control from raw input to system controls.
     *
     * @param setpointDeltaMul the multiplicative scale to translate power into target position delta, which returns
     *                         the desired delta step in encoder ticks, <b>while supplying you with a delta time (dt) in seconds</b>.
     *                         Delta time is calculated as the time between the last two evaluations of the function. It can
     *                         be used to define a rate of change in the setpoint with respect to time rather than loop times.
     *                         E.g. 100 ticks per second ({@code (dt) -> 100 * dt}).
     * @return this
     * @see #disableOvercurrent()
     */
    @NonNull
    public HoldableActuator withUserSetpointControl(@Nullable UnaryFunction setpointDeltaMul) {
        userSetpointControl = setpointDeltaMul;
        if (userSetpointControl != null && inputMode == Mode.USER_POWER)
            inputMode = Mode.USER_SETPOINT;
        return this;
    }

    /**
     * Calling this method will restore the user input functionality translating into direct power on the motor.
     *
     * @return this
     * @see #withUserSetpointControl(UnaryFunction)
     */
    @NonNull
    public HoldableActuator disableUserSetpointControl() {
        userSetpointControl = null;
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
        userPower = Mathf.clamp(p, lowerPower, upperPower);
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
                motorPower = rtpPower * Math.signum(target - current);
                opMode(o -> o.telemetry.add("%: <font color='#FF5F1F'>MOVING -> %/% ticks</font> [%tps]", this, current, target, Math.round(encoder.getVelocity())));
                break;
            case HOMING:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorPower = homePower * Math.signum(homingDirection);
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
                    motorPower = rtpPower * Math.signum(target - current);
                } else {
                    userLatch = false;
                    // Move arm in accordance with the user's input
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorPower = userPower;
                }
                opMode(o -> o.telemetry.add("%: % at % ticks [%tps]", this, userPower == 0.0 ? "<font color='green'>HOLDING</font>" : "<font color='#FF5F1F'><b>MOVING</b></font>", current, Math.round(encoder.getVelocity())));
                break;
            case USER_SETPOINT:
                double dt;
                if (opMode != null) {
                    dt = opMode.timer.deltaTime().in(Seconds);
                } else {
                    double now = System.nanoTime() / 1.0E9;
                    if (lastTime == -1)
                        lastTime = now;
                    dt = now - lastTime;
                    lastTime = now;
                }
                newTarget = target + userPower * userSetpointControl.apply(dt);
                motorPower = rtpPower * Math.signum(target - current);
                opMode(o -> o.telemetry.add("%: % at % ticks, % error [%tps]", this, !motor.isBusy() ? "<font color='green'>SUSTAINING</font>" : "<font color='#FF5F1F'><b>RESPONDING</b></font>", current, target - current, Math.round(encoder.getVelocity())));
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
                encoder.reset();
                motorPower = 0;
                if (target < 0)
                    newTarget = 0;
            }

            if (bottomSwitch.isPressed() && !zeroed) {
                DcMotor.RunMode prev = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.reset();
                // Must propagate now as we're switching the mode
                sustainedTolerated.reset();
                newTarget = 0;
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

        if (inputMode != Mode.HOMING && ((current < minLimit && motorPower < 0.0) || (current > maxLimit && motorPower > 0.0))) {
            // Cancel any tasks that would move the actuator out of bounds by autonomous operation
            setInputModeToUser();
            motorPower = 0.0;
        }

        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            boolean overCurrent = motor.isOverCurrent();
            if (sustainedOvercurrent.seconds() >= overcurrentTime.in(Seconds) && overCurrent) {
                Dbg.warn(getClass(), "%Warning: Stall detection (continued % A for % sec) has been activated. To prevent motor damage, the target position has been auto set to the current position (% -> %).", isDefaultName() ? "" : "(" + name + ") ", Mathf.round(motor.getCurrentAlert(CurrentUnit.AMPS), 1), Mathf.round(overcurrentTime.in(Seconds), 1), target, current);
                newTarget = motor.getCurrentPosition();
            } else if (!overCurrent) {
                sustainedOvercurrent.reset();
            }

            if (maxSteadyState != null) {
                // Steady state error will be if we're not near the target and not moving in any meaningful quantity
                if (Mathf.isNear(current, target, tolerance) || !Mathf.isNear(encoder.getVelocity(), 0, tolerance)) {
                    sustainedTolerated.reset();
                }
                if (sustainedTolerated.seconds() >= maxSteadyState.in(Seconds)) {
                    Dbg.warn(getClass(), "%Warning: Steady state error has been detected for % sec. To prevent motor damage, the target position has been auto set to the current position (% -> %).", isDefaultName() ? "" : "(" + name + ") ", Mathf.round(maxSteadyState.in(Seconds), 1), target, current);
                    newTarget = motor.getCurrentPosition();
                }
            }
        }

        if (newTarget != -1) {
            motor.setTargetPosition((int) Math.round(Mathf.clamp(newTarget, minLimit, maxLimit)));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(Mathf.clamp(motorPower, lowerPower, upperPower));
    }

    @Override
    protected void onDisable() {
        motor.setPower(0);
    }

    private void setInputModeToUser() {
        userLatch = false;
        inputMode = userSetpointControl != null ? Mode.USER_SETPOINT : Mode.USER_POWER;
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
                    .withName(name + ":Set Power");
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
            return Task.task()
                    .init(HoldableActuator.this::setInputModeToUser)
                    .loop(() -> userPower = pwr)
                    .finish(() -> userPower = 0)
                    .on(HoldableActuator.this)
                    .override(true)
                    .timeout(time)
                    .name(name + ":Run For Time")
                    .build();
        }

        /**
         * Home the actuator based on encoders against a hard stop or limit switch. This task ignores
         * the lower and upper limits as defined by this class.
         *
         * @return a task to home the actuator
         */
        @NonNull
        public Task home() {
            return new Task(homingTimeout) {
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
                    zeroHits = 0;
                    homingDirection = -1;
                    inputMode = Mode.HOMING;
                }

                @Override
                protected void periodic() {
                    if (zeroHitThreshold <= 0) return;
                    if (encoder.getVelocity() >= 0) {
                        zeroHits++;
                    } else {
                        zeroHits = 0;
                    }
                }

                @Override
                protected void onFinish() {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setInputModeToUser();
                }

                @Override
                protected boolean isTaskFinished() {
                    boolean bottomedOut = bottomSwitch != null && bottomSwitch.isPressed();
                    boolean velocityZeroed = zeroHitThreshold > 0 && zeroHits >= zeroHitThreshold;
                    boolean overCurrent = overcurrentTime.magnitude() > 0 && motor.isOverCurrent();
                    if (overcurrentTime.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                        overcurrentTimer = new ElapsedTime();
                    } else if (!overCurrent) {
                        overcurrentTimer = null;
                    }
                    boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= overcurrentTime.in(Seconds);
                    return inputMode != Mode.HOMING || (bottomedOut || velocityZeroed || sustainedOvercurrent);
                }
            }.onSubsystem(HoldableActuator.this, true).withName(name + ":Return To Home");
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
            return new Task(homingTimeout) {
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
                    zeroHits = 0;
                    homingDirection = 1;
                    inputMode = Mode.HOMING;
                }

                @Override
                protected void periodic() {
                    if (zeroHitThreshold <= 0) return;
                    if (encoder.getVelocity() <= 0) {
                        zeroHits++;
                    } else {
                        zeroHits = 0;
                    }
                }

                @Override
                protected void onFinish() {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setInputModeToUser();
                }

                @Override
                protected boolean isTaskFinished() {
                    boolean toppedOut = topSwitch != null && topSwitch.isPressed();
                    boolean velocityZeroed = zeroHitThreshold > 0 && zeroHits >= zeroHitThreshold;
                    boolean overCurrent = overcurrentTime.magnitude() > 0 && motor.isOverCurrent();
                    if (overcurrentTime.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                        overcurrentTimer = new ElapsedTime();
                    } else if (!overCurrent) {
                        overcurrentTimer = null;
                    }
                    boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= overcurrentTime.in(Seconds);
                    return inputMode != Mode.HOMING || (toppedOut || velocityZeroed || sustainedOvercurrent);
                }
            }.onSubsystem(HoldableActuator.this, true).withName(name + ":Travel To Ceiling");
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
            return goTo(position).until(limitSwitch::isPressed).withName(name + ":Run To Limit Switch");
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
                    sustainedTolerated.reset();
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
                    return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(targetPosition, encoder.getPosition(), tolerance));
                }
            }.onSubsystem(HoldableActuator.this, true).withName(name + ":Run To " + targetPosition + " Ticks");
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
                    sustainedTolerated.reset();
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
                    return inputMode != Mode.AUTO || (!motor.isBusy() && Mathf.isNear(target, encoder.getPosition(), tolerance));
                }
            }.onSubsystem(HoldableActuator.this, true).withName(name + ":Run To " + deltaPosition + " Delta Ticks");
        }
    }
}
