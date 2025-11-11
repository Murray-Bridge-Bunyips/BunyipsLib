package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.TrapezoidProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.DualServos;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.Switch;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Extension of the extended {@link Servo} interface that allows for motion profiling via a {@link TrapezoidProfile}.
 * This extension also offers refresh rate and position cache tolerance handling for loop time
 * optimisation similar to an {@link SimpleRotator}.
 * <p>
 * This class serves as a drop-in replacement for the {@link Servo}, similar to {@link Motor} with the {@link DcMotor}.
 * Do note that this class cannot be cast to a {@link ServoImplEx} instance, but it does implement the extended
 * {@link PwmControl} interface for extended operations.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class ServoEx extends ServoImpl implements PwmControl {
    @Nullable
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double lastDtSec = -1;

    private double positionDeltaTolerance;
    private double lastPosition = -1;
    private long refreshRateNanos;
    private long lastUpdate;
    private boolean triedInit;

    private Measure<Time> endToEndTime = Milliseconds.of(500);

    /**
     * Wrap a Servo to use with the ServoEx class.
     *
     * @param servo the Servo from hardwareMap to use.
     */
    public ServoEx(@NonNull Servo servo) {
        super(servo.getController(), servo.getPortNumber(), servo.getDirection());
    }

    /**
     * Utility for debugging a servo on telemetry/logs with current target position information. This will log
     * to all telemetry sources and log to the {@link FlightRecorder}.
     * <p>
     * Must be called periodically.
     *
     * @param servo The servo to debug
     * @param name  The name to debug as
     */
    public static void debug(@NonNull Servo servo, @NonNull String name) {
        debug(servo, name, false);
    }

    /**
     * Utility for debugging a servo on telemetry/logs with current target position information.
     * <p>
     * Must be called periodically.
     *
     * @param servo   The servo to debug
     * @param name    The name to debug as
     * @param onlyLog Whether to only record debug information to the {@link FlightRecorder}, not telemetry/live dash.
     */
    public static void debug(@NonNull Servo servo, @NonNull String name, boolean onlyLog) {
        int port = servo.getPortNumber();
        double target = servo.getPosition();
        String channelPrefix = Text.format("SERVO_P%_%_", port, Text.upper(name).replace(" ", "_"));
        if (!onlyLog)
            DualTelemetry.smartAdd(Text.format("% Target (0-1, port %)", name, port), target);
        FlightRecorder.write(channelPrefix + "TARGET", target);
        if (servo instanceof ServoEx sex) {
            double lastTarget = sex.lastPosition;
            if (!onlyLog)
                DualTelemetry.smartAdd(Text.format("% Last Target (0-1, port %)", name, port), lastTarget);
            FlightRecorder.write(channelPrefix + "LAST_TARGET", lastTarget);
        }
    }

    /**
     * Attempts to get the end-to-end time for this servo (only available on ServoEx).
     *
     * @param servo the servo to get the time for (rescaling parameters are ignored if this is not a ServoEx)
     * @param from  the rescaling lower limit where the servo is currently
     * @param to    the rescaling upper limit where the servo should go to
     * @return the rescaled servo end-to-end time if available, else {@link Lambda#EPSILON_MS}.
     */
    public static Measure<Time> tryGetEndToEndTime(Servo servo, double from, double to) {
        return servo instanceof ServoEx sex ? sex.endToEndTime.times(to - from) : Milliseconds.of(Lambda.EPSILON_MS);
    }

    /**
     * Set the delta in servo position required to propagate a hardware write.
     *
     * @param magnitude absolute magnitude of delta in servo position, 0/default will disable
     */
    public void setPositionDeltaThreshold(double magnitude) {
        positionDeltaTolerance = Math.abs(magnitude);
    }

    /**
     * Set the refresh rate of the servo that will be a minimum time between hardware writes.
     * Consistent calls to {@link #setPosition(double)} is required for this refresh rate to be effective.
     *
     * @param refreshRate the refresh rate interval, <=0/default will disable
     */
    public void setPositionRefreshRate(@NonNull Measure<Time> refreshRate) {
        refreshRateNanos = (long) refreshRate.in(Nanoseconds);
    }

    /**
     * The user-defined time it takes for this servo to travel from position 0 to 1 or 1 to 0.
     * Default of 1 second.
     *
     * @return end to end time
     */
    @NonNull
    public Measure<Time> getEndToEndTime() {
        return endToEndTime;
    }

    /**
     * Set the time at which it takes the servo to complete a full rotation from the programmed 0 to 1 position (and
     * vice versa). This value is used to define the task timing for several subsystems, including the {@link Switch}
     * and {@link DualServos}.
     * <p>
     * This value can also be used for your own purposes accessed through {@link #getEndToEndTime()}.
     *
     * @param servoFullRotationTime the time it takes for the servo to travel from programmed position 0 to 1 or 1 to 0.
     */
    public void setEndToEndTime(@NonNull Measure<Time> servoFullRotationTime) {
        endToEndTime = servoFullRotationTime;
    }

    /**
     * Sets the trapezoidal constraints to apply to this servo's positions. These constraints are in units
     * of delta step of position (for example, 1 corresponds with the full servo travel distance per second).
     *
     * @param positionConstraints the position velocity and acceleration constraints
     */
    public void setConstraints(@Nullable TrapezoidProfile.Constraints positionConstraints) {
        constraints = positionConstraints;
    }

    /**
     * Return to standard servo controls without a motion profile. This is shorthand for {@code setConstraints(null)}.
     */
    public void disableConstraints() {
        constraints = null;
    }

    /**
     * Sets the current position of the servo, expressed as a fraction of its available
     * range. If PWM power is enabled for the servo, the servo will attempt to move to
     * the indicated position.
     * <p>
     * <b>Important ServoEx Note:</b> Since this class requires continuous update with a motion profile,
     * it is important that this method is being called periodically if one is being used; this is similar
     * to how {@code setPower} has to be called periodically in {@link Motor} to update system controllers.
     *
     * @param targetPosition the position to which the servo should move, a value in the range [0.0, 1.0]
     * @see ServoController#pwmEnable()
     * @see #getPosition()
     */
    @Override
    public synchronized void setPosition(double targetPosition) {
        if (constraints != null) {
            // Apply motion profiling for current target in seconds
            TrapezoidProfile.State goal = new TrapezoidProfile.State(targetPosition, 0);
            TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
            double t = System.nanoTime() / 1.0E9;
            if (lastDtSec == -1) lastDtSec = t;
            setpoint = profile.calculate(t - lastDtSec);
            lastDtSec = t;
            targetPosition = setpoint.position;
        }

        // Apply refresh rate and cache restrictions
        long now = System.nanoTime();
        if (refreshRateNanos > 0 && Math.abs(lastUpdate - now) < refreshRateNanos) {
            return;
        }
        if (Math.abs(lastPosition - targetPosition) < positionDeltaTolerance && targetPosition != 1 && targetPosition != 0) {
            return;
        }
        if (targetPosition == lastPosition) {
            // Useless operation
            return;
        }

        lastUpdate = now;
        lastPosition = targetPosition;

        if (!triedInit && getController() instanceof LynxServoController lynx) {
            // Paranoia, servos sometimes act strangely and don't reset or update first state properly
            // Calling set position on init may not do anything due to internal cache
            // We ensure we give clear direct initial instructions to the servo to energise properly
            lynx.forgetLastKnown();
            // Scales properly if we use the super method rather than the controller itself
            super.setPosition(targetPosition);
            lynx.setServoPwmEnable(getPortNumber());
            triedInit = true;
            return;
        }

        super.setPosition(targetPosition);
    }

    // Servo configuration type cannot be accessed from this class, so we'll need to implement the extended interface
    // methods manually - we can't actually extend ServoImplEx. The servo configuration type is not exposed but is
    // important for operations, so we won't try to feed the configuration fake data.

    /**
     * Returns the current PWM range limits for the servo
     *
     * @return the current PWM range limits for the servo
     * @see #setPwmRange(PwmRange)
     */
    @NonNull
    @Override
    public PwmRange getPwmRange() {
        return ((ServoControllerEx) getController()).getServoPwmRange(getPortNumber());
    }

    /**
     * Sets the PWM range limits for the servo
     *
     * @param range the new PWM range limits for the servo
     * @see #getPwmRange()
     */
    @Override
    public void setPwmRange(@NonNull PwmRange range) {
        ((ServoControllerEx) getController()).setServoPwmRange(getPortNumber(), range);
    }

    /**
     * Individually energizes the PWM for this particular servo.
     *
     * @see #setPwmDisable()
     * @see #isPwmEnabled()
     */
    @Override
    public void setPwmEnable() {
        ((ServoControllerEx) getController()).setServoPwmEnable(getPortNumber());
    }

    /**
     * Individually de-energizes the PWM for this particular servo
     *
     * @see #setPwmEnable()
     */
    @Override
    public void setPwmDisable() {
        ((ServoControllerEx) getController()).setServoPwmDisable(getPortNumber());
    }

    /**
     * Returns whether the PWM is energized for this particular servo
     *
     * @see #setPwmEnable()
     */
    @Override
    public boolean isPwmEnabled() {
        return ((ServoControllerEx) getController()).isServoPwmEnabled(getPortNumber());
    }
}
