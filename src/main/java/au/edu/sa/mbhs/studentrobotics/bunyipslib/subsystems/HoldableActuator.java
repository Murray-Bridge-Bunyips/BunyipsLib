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
import java.util.Objects;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Current;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Encoder;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Exceptions;

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
    // Hardware
    private final HashMap<TouchSensor, Integer> switchMapping = new HashMap<>();
    // Safety and hardware protections
    private final ElapsedTime sustainedOvercurrent = new ElapsedTime();
    private final ElapsedTime sustainedTolerated = new ElapsedTime();
    // Operation modes
    private final BidirectionalHomingTask.Parameters homingParameters = new BidirectionalHomingTask.Parameters();
    private final UserPowerControl upc = new UserPowerControl();
    // In v5.1.0+ user setpoint control has been offered to instead interpret power as a multiple of some delta-time
    // multiplied constant which allows finer control over the setpoint rather than feeding raw power.
    private final UserSetpointControl usc = new UserSetpointControl();
    private double power;
    private TouchSensor topSwitch;
    private TouchSensor bottomSwitch;
    private DcMotorEx motor;
    private Encoder encoder;
    private Measure<Time> overcurrentTime;
    private Measure<Time> maxSteadyState;
    private boolean autoZeroingLatch;
    // Clampings
    private long minLimit = -Long.MAX_VALUE;
    private long maxLimit = Long.MAX_VALUE;
    private double lowerPower = -1.0;
    private double upperPower = 1.0;
    private double autoPower = 1.0;
    private UnaryFunction userSetpointControl;

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
        this.motor.setPower(autoPower);
        // Encoder instance is used for awareness of the encoder position, we generally don't care about direction
        // as it is handled by the motor object itself, unless we're using a Motor object then we can conveniently
        // hook into it
        if (motor instanceof Motor) {
            encoder = ((Motor) motor).encoder;
        } else {
            encoder = new Encoder(this.motor::getCurrentPosition, this.motor::getVelocity);
        }
        // We manage caching manually
        encoder.setCaching(true);
        // Sane default, will need adjusting manually
        withOvercurrent(Amps.of(8), Seconds.of(2));
    }

    /**
     * Set the target tolerance of the actuator.
     * <p>
     * Applies directly to the motor object.
     *
     * @param tolerance the tolerance to set in encoder ticks
     * @return this
     */
    @NonNull
    public HoldableActuator withTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
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
        homingParameters.zeroHitThreshold = threshold;
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
     * If this current is reached during a state where stalling is monitored for the set duration set here, the actuator
     * will auto-reset the motor.
     *
     * @param current the current threshold where exceeding will intervene. Default is 8A.
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
        homingParameters.homingTimeout = timeout;
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
     * Set the top limit switch of the actuator to use in encoder awareness and the ceiling task.
     *
     * @param topLimitSwitch the limit switch to set as the top switch where the arm would be at the max position
     * @return this
     */
    @NonNull
    public HoldableActuator withTopSwitch(@Nullable TouchSensor topLimitSwitch) {
        if (topLimitSwitch == null)
            sout(Dbg::error, "Supplied top limit switch is null! Ignoring.");
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
            sout(Dbg::error, "Supplied bottom limit switch is null! Ignoring.");
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
     * Set the auto response power magnitude of the actuator.
     * <p>
     * If positive power is supplied to the motor, the mechanism should be brought "upwards" (away from the bottom
     * relative to your limit switches and homing direction).
     * <p>
     * This value is used to scale automatic responses by the actuator. Default of 1.0.
     * <p>
     * Note: This power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set (magnitude)
     * @return this
     */
    @NonNull
    public HoldableActuator withAutoPower(double targetPower) {
        autoPower = Math.abs(targetPower);
        return this;
    }

    /**
     * Set the homing power magnitude of the actuator.
     * <p>
     * If homing power is supplied to the motor, the mechanism should be brought "downwards" (away from the upwards limit
     * relative to your limit switches and maximum constraints). An inverted version is used for the ceiling task.
     * <p>
     * This power will be used as the raw power in a home or ceiling task. Default of 0.7.
     * <p>
     * Note: This power is clamped by the lower and upper power clamps.
     *
     * @param targetPower the power to set (magnitude)
     * @return this
     */
    @NonNull
    public HoldableActuator withHomingPower(double targetPower) {
        homingParameters.homePower = Math.abs(targetPower);
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
     * This mode is useful to call on high-frequency system controllers, like those accomplished via {@link Motor},
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
     * <p>
     * <b>Note:</b> This method is ignored when a task other than the default is running. This is intentional behaviour
     * to avoid power conflicts.
     *
     * @param p power level in domain [-1.0, 1.0], will be clamped
     * @return this
     */
    @NonNull
    public HoldableActuator setPower(double p) {
        if (isRunningDefaultTask())
            power = Mathf.clamp(p, lowerPower, upperPower);
        return this;
    }

    @Override
    protected void periodic() {
        double pwr = power;
        encoder.clearCache();

        // Limit switch position rebounding
        for (TouchSensor limitSwitch : switchMapping.keySet()) {
            // Map encoder to limit switch positions
            if (limitSwitch.isPressed()) {
                Integer ticks = switchMapping.get(limitSwitch);
                if (ticks != null)
                    encoder.setKnownPosition(ticks);
            }
        }

        // Bottom limit switch protection
        if (bottomSwitch != null) {
            if (bottomSwitch.isPressed() && pwr < 0) {
                // Cancel and stop any tasks that would move the actuator out of bounds as defined by the limit switch
                cancelCurrentTask();
                encoder.reset();
                pwr = 0;
                if (motor.getTargetPosition() < 0)
                    motor.setTargetPosition(0);
            }

            if (bottomSwitch.isPressed() && !autoZeroingLatch) {
                DcMotor.RunMode prev = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.reset();
                // Must propagate now as we're switching the mode
                sustainedTolerated.reset();
                motor.setTargetPosition(0);
                motor.setMode(prev);
                // Ensure we only run the reset once every time the switch is pressed
                autoZeroingLatch = true;
            }

            if (!bottomSwitch.isPressed())
                autoZeroingLatch = false;
        }

        // Top limit switch protection
        if (topSwitch != null && topSwitch.isPressed() && pwr > 0) {
            cancelCurrentTask();
            pwr = 0;
            int current = encoder.getPosition();
            if (motor.getTargetPosition() > current)
                motor.setTargetPosition(current);
        }

        // Unusual state that could lead to infinite homing
        if (topSwitch != null && bottomSwitch != null && topSwitch.isPressed() && bottomSwitch.isPressed()) {
            sout(Dbg::warn, "Warning: Both limit switches were pressed at the same time. This indicates an impossible system state.");
        }

        // Bounds protection when the actuator is not homing
        if (!(getCurrentTask() instanceof BidirectionalHomingTask)) {
            int currentPosition = encoder.getPosition();
            if ((currentPosition < minLimit && pwr < 0.0) || (currentPosition > maxLimit && pwr > 0.0)) {
                // Try to cancel any tasks that would move the actuator out of bounds by autonomous operation
                cancelCurrentTask();
                // We also set the power to 0 to nullify any user input
                pwr = 0;
            }
        }

        // Autonomous protections
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // Stall detection
            boolean overCurrent = motor.isOverCurrent();
            if (sustainedOvercurrent.seconds() >= overcurrentTime.in(Seconds) && overCurrent) {
                sout(Dbg::warn, "Warning: Stall detection (continued % A for % sec) has been activated. To prevent motor damage, the target position has been auto set to the current position (% -> %).", Mathf.round(motor.getCurrentAlert(CurrentUnit.AMPS), 1), Mathf.round(overcurrentTime.in(Seconds), 1), motor.getTargetPosition(), encoder.getPosition());
                motor.setTargetPosition(encoder.getPosition());
            } else if (!overCurrent) {
                sustainedOvercurrent.reset();
            }

            // Steady-state detection
            if (maxSteadyState != null) {
                // Steady state error will be if we're not near the target and not moving in any meaningful quantity
                int tolerance = motor.getTargetPositionTolerance();
                if (Mathf.isNear(encoder.getPosition(), motor.getTargetPosition(), tolerance) || !Mathf.isNear(encoder.getVelocity(), 0, tolerance)) {
                    sustainedTolerated.reset();
                }
                if (sustainedTolerated.seconds() >= maxSteadyState.in(Seconds)) {
                    sout(Dbg::warn, "Warning: Steady state error has been detected for % sec. To prevent motor damage, the target position has been auto set to the current position (% -> %).", Mathf.round(maxSteadyState.in(Seconds), 1), motor.getTargetPosition(), encoder.getPosition());
                    motor.setTargetPosition(encoder.getPosition());
                }
            }
        }

        if (isRunningDefaultTask()) {
            // Manual user control
            if (userSetpointControl != null)
                usc.accept(pwr);
            else
                upc.accept(pwr);
        }

        motor.setPower(Mathf.clamp(pwr, lowerPower, upperPower));
    }

    @Override
    protected void onEnable() {
        // HoldableActuator will handle the encoder reports if we're on a Motor class
        // Technically not required if we have bulk reads but may still occur and we want to be positively sure
        // we aren't reading more than we need to be
        encoder.setCaching(true);
    }

    @Override
    protected void onDisable() {
        power = 0;
        motor.setPower(0);
        encoder.setCaching(false);
    }

    // Internal "tasks" that define user operations as executed by periodic. We can't really use Tasks for runtime via the user
    // since we need to preserve behaviours in non-task environments to respect user input to follow setpoints or by power. The
    // next best option for cleanliness is we do this and isolate the contexts, handling .run() ourselves. This does not apply
    // to the goTo or Home tasks as they are mandatory operations to use tasks. We're only concerned about setPower().

    private class UserPowerControl implements DoubleConsumer {
        private boolean zeroInputLatch;

        @Override
        public void accept(double pwr) {
            if (pwr == 0.0) {
                // Hold actuator in place
                if (!zeroInputLatch) {
                    motor.setTargetPosition(motor.getCurrentPosition());
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    zeroInputLatch = true;
                }
                power = autoPower * Math.signum(motor.getTargetPosition() - encoder.getPosition());
            } else {
                zeroInputLatch = false;
                // Follow user input
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                power = pwr;
            }
            DualTelemetry.smartAdd(HoldableActuator.this.toString(), "% at % ticks [%tps]", pwr == 0.0 ? "<font color='green'>HOLDING</font>" : "<font color='#FF5F1F'><b>MOVING</b></font>", encoder.getPosition(), Math.round(encoder.getVelocity()));
        }
    }

    private class UserSetpointControl implements DoubleConsumer {
        private double lastTime = -1;

        @Override
        public void accept(double pwr) {
            double now = System.nanoTime() / 1.0E9;
            if (lastTime == -1)
                lastTime = now;
            double dt = now - lastTime;
            lastTime = now;

            int current = encoder.getPosition();
            double target = motor.getTargetPosition() + pwr * Objects.requireNonNull(userSetpointControl).apply(dt);

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(Math.round((float) target));

            power = autoPower * Math.signum(target - current);

            DualTelemetry.smartAdd(HoldableActuator.this.toString(), "% at % ticks [%tps], % error", !motor.isBusy() ? "<font color='green'>SUSTAINING</font>" : "<font color='#FF5F1F'><b>RESPONDING</b></font>", current, Math.round(encoder.getVelocity()), Math.round(Math.abs(target - current)));
        }
    }

    /**
     * Standard homing task that moves the motor in a direction until it hits a hard stop, the velocity becomes
     * too slow, or overcurrent is detected for a period of time.
     * <p>
     * Available through {@code tasks.home()} and {@code tasks.ceil()}.
     */
    public class BidirectionalHomingTask extends Task {
        private final int homingDirection;
        private final TouchSensor targetSwitch;
        private ElapsedTime overcurrentTimer;
        private double zeroHits;

        /**
         * Create a new BidirectionalHomingTask.
         * <p>
         * This task is also available through {@code tasks.home()} and {@code tasks.ceil()}.
         *
         * @param direction the direction the arm should move (1 to ceil, -1 to home)
         */
        public BidirectionalHomingTask(int direction) {
            if (direction != 1 && direction != -1)
                throw new Exceptions.EmergencyStop("Invalid direction index: " + direction + ", must be -1 or 1!");
            timeout = homingParameters.homingTimeout;
            on(HoldableActuator.this, true);
            homingDirection = direction;
            named(name + (homingDirection == -1 ? ":Return To Home" : ":Travel To Ceiling"));
            targetSwitch = homingDirection == -1 ? bottomSwitch : topSwitch;
        }

        @Override
        protected void init() {
            // Stop now if the switch is already pressed
            if (targetSwitch != null && targetSwitch.isPressed()) {
                finishNow();
                return;
            }
            zeroHits = 0;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        protected void periodic() {
            if (homingParameters.zeroHitThreshold >= 0) {
                if (homingDirection == -1 ? encoder.getVelocity() >= 0 : encoder.getVelocity() <= 0) {
                    zeroHits++;
                } else {
                    zeroHits = 0;
                }
            }
            power = homingParameters.homePower * Math.signum(homingDirection);
            DualTelemetry.smartAdd(HoldableActuator.this.toString(), "<font color='yellow'><b>HOMING</b></font> [%tps]", Math.round(encoder.getVelocity()));
        }

        @Override
        protected void onFinish() {
            power = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.reset();
        }

        @Override
        protected boolean isTaskFinished() {
            boolean hardStop = targetSwitch != null && targetSwitch.isPressed();
            boolean velocityZeroed = homingParameters.zeroHitThreshold > 0 && zeroHits >= homingParameters.zeroHitThreshold;
            boolean overCurrent = overcurrentTime.magnitude() > 0 && motor.isOverCurrent();
            if (overcurrentTime.magnitude() > 0 && overCurrent && overcurrentTimer == null) {
                overcurrentTimer = new ElapsedTime();
            } else if (!overCurrent) {
                overcurrentTimer = null;
            }
            boolean sustainedOvercurrent = overcurrentTimer != null && overcurrentTimer.seconds() >= overcurrentTime.in(Seconds);
            return hardStop || velocityZeroed || sustainedOvercurrent;
        }

        private static class Parameters {
            // Sane defaults
            public Measure<Time> homingTimeout = Seconds.of(5);
            public double homePower = 0.7;
            public int zeroHitThreshold = 30;
        }
    }

    /**
     * Tasks for HoldableActuator, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Controls the actuator with a supplier of power.
         * <p>
         * <b>Note:</b> Should be allocated as a default task, otherwise this task will not trigger the auto-capturing
         * power setters and nothing will happen.
         * <p>
         * Power will be translated accordingly depending on the state of {@link #withUserSetpointControl(UnaryFunction)}
         * and if it has been called.
         *
         * @param powerSupplier the power value supplier
         * @return a task to move the actuator
         */
        @NonNull
        public Task control(@NonNull DoubleSupplier powerSupplier) {
            // Although we could allocate this task as a default task ourselves, we want to have the user
            // do it as we don't want to enforce behaviours in the event other behaviour is wanted.
            return Task.task().periodic(() -> HoldableActuator.this.setPower(powerSupplier.getAsDouble()))
                    .onFinish(() -> power = 0)
                    .on(HoldableActuator.this, false)
                    // It's fine for this task name to be stale since we shouldn't expect dynamic disabling of sp. ctrl.
                    .named(userSetpointControl != null ? "Setpoint Delta Control" : "Power Target Control");
        }

        /**
         * Directly set the power of the actuator.
         *
         * @param p the power to set
         * @return a task to set the power
         */
        @NonNull
        public Task setPower(double p) {
            return new Lambda(() -> power = Mathf.clamp(p, -1, 1))
                    .on(HoldableActuator.this, false)
                    .named(forThisSubsystem("Set Power to " + p));
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
                    .init(() -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
                    .periodic(() -> power = pwr)
                    .onFinish(() -> power = 0)
                    .on(HoldableActuator.this, true)
                    .timeout(time)
                    .named(forThisSubsystem("Run For " + time));
        }

        /**
         * Home the actuator based on encoders against a hard stop or limit switch. This task ignores
         * the lower and upper limits as defined by this class.
         *
         * @return a task to home the actuator
         */
        @NonNull
        public Task home() {
            return new BidirectionalHomingTask(-1);
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
            return new BidirectionalHomingTask(1);
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
                sout(Dbg::error, "Attempted to go to a limit switch that was not mapped. This task will not run.");
                return new Lambda();
            }
            // Since this is a static mapping we can return the task
            return goTo(position).until(limitSwitch::isPressed).named(forThisSubsystem("Run To Limit Switch"));
        }

        /**
         * Commands the actuator to go to a particular encoder tick reading by advancing or retreating the motor.
         * <p>
         * Informally known as the Doinky-Rubber-Bandy Task.
         *
         * @param targetPosition the position to set
         * @return a task to set the position
         */
        @NonNull
        public Task goTo(int targetPosition) {
            return Task.task()
                    .init(() -> {
                        sustainedTolerated.reset();
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setTargetPosition(targetPosition);
                    })
                    .periodic(() -> {
                        int target = motor.getTargetPosition();
                        int current = encoder.getPosition();
                        power = autoPower * Math.signum(target - current);
                        DualTelemetry.smartAdd(HoldableActuator.this.toString(), "<font color='#FF5F1F'>MOVING -> %/% ticks</font> [%tps]", current, target, Math.round(encoder.getVelocity()));
                    })
                    .onFinish(() -> power = 0)
                    .isFinished(() -> !motor.isBusy() && Mathf.isNear(targetPosition, encoder.getPosition(), motor.getTargetPositionTolerance()))
                    .on(HoldableActuator.this, true)
                    .named(forThisSubsystem("Run To " + targetPosition + " Ticks"));
        }

        /**
         * Commands the actuator to go to a particular encoder position as an offset of the current position when
         * this task is executed.
         *
         * @param deltaPosition the delta to add to the current position of the actuator at task runtime
         * @return a task to delta the position
         */
        @NonNull
        public Task delta(int deltaPosition) {
            return Task.defer(() -> goTo(encoder.getPosition() + deltaPosition))
                    .named(forThisSubsystem("Run To " + deltaPosition + " Delta Ticks"));
        }
    }
}
