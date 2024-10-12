package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.TrapezoidProfile;

/**
 * Extension of the extended {@link Servo} interface that allows for motion profiling via a {@link TrapezoidProfile}.
 * <p>
 * This class serves as a drop-in replacement for the {@link Servo}, similar to {@link Motor} with the {@link DcMotor}.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class ProfiledServo implements Servo, PwmControl {
    protected final ServoControllerEx controller;
    protected final int port;
    private final String deviceName;

    protected Direction direction = Direction.FORWARD;
    protected double limitPositionMin = MIN_POSITION;
    protected double limitPositionMax = MAX_POSITION;

    @Nullable
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double lastDt = -1;

    /**
     * Sets the trapezoidal constraints to apply to this servo's positions. These constraints are in units
     * of delta step of position.
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
     * Wrap a Servo to use with the ProfiledServo class.
     *
     * @param servo the Servo from hardwareMap to use.
     */
    public ProfiledServo(Servo servo) {
        controller = (ServoControllerEx) servo.getController();
        port = servo.getPortNumber();
        deviceName = servo.getDeviceName();
    }

    /**
     * Sets the PWM range limits for the servo
     *
     * @param range the new PWM range limits for the servo
     * @see #getPwmRange()
     */
    @Override
    public void setPwmRange(PwmRange range) {
        controller.setServoPwmRange(port, range);
    }

    /**
     * Returns the current PWM range limits for the servo
     *
     * @return the current PWM range limits for the servo
     * @see #setPwmRange(PwmRange)
     */
    @Override
    public PwmRange getPwmRange() {
        return controller.getServoPwmRange(port);
    }

    /**
     * Individually energizes the PWM for this particular servo.
     *
     * @see #setPwmDisable()
     * @see #isPwmEnabled()
     */
    @Override
    public void setPwmEnable() {
        controller.setServoPwmEnable(port);
    }

    /**
     * Individually denergizes the PWM for this particular servo
     *
     * @see #setPwmEnable()
     */
    @Override
    public void setPwmDisable() {
        controller.setServoPwmDisable(port);
    }

    /**
     * Returns whether the PWM is energized for this particular servo
     *
     * @see #setPwmEnable()
     */
    @Override
    public boolean isPwmEnabled() {
        return controller.isServoPwmEnabled(port);
    }

    /**
     * Returns the underlying servo controller on which this servo is situated.
     *
     * @return the underlying servo controller on which this servo is situated.
     * @see #getPortNumber()
     */
    @Override
    public ServoController getController() {
        return controller;
    }

    /**
     * Returns the port number on the underlying servo controller on which this motor is situated.
     *
     * @return the port number on the underlying servo controller on which this motor is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return port;
    }

    /**
     * Sets the logical direction in which this servo operates.
     *
     * @param direction the direction to set for this servo
     * @see #getDirection()
     * @see Direction
     */
    @Override
    public synchronized void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Returns the current logical direction in which this servo is set as operating.
     *
     * @return the current logical direction in which this servo is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return direction;
    }

    /**
     * Sets the current position of the servo, expressed as a fraction of its available
     * range. If PWM power is enabled for the servo, the servo will attempt to move to
     * the indicated position.
     * <p>
     * <b>Important ProfiledServo Note:</b> Since this class requires continuous update with a motion profile,
     * it is important that this method is being called periodically if one is being used; this is similar
     * to how {@code setPower} has to be called periodically in {@link Motor} to update system controllers.
     *
     * @param targetPosition the position to which the servo should move, a value in the range [0.0, 1.0]
     * @see ServoController#pwmEnable()
     * @see #getPosition()
     */
    @Override
    public synchronized void setPosition(double targetPosition) {
        targetPosition = Mathf.clamp(targetPosition, MIN_POSITION, MAX_POSITION);
        if (direction == Direction.REVERSE) targetPosition = reverse(targetPosition);
        targetPosition = Mathf.scale(targetPosition, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);

        if (constraints != null) {
            goal = new TrapezoidProfile.State(targetPosition, 0);
            TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
            double t = System.nanoTime() / 1.0E9;
            if (lastDt == -1) lastDt = t;
            setpoint = profile.calculate(t - lastDt);
            lastDt = t;
            targetPosition = setpoint.position;
        }

        controller.setServoPosition(port, targetPosition);
    }

    /**
     * Returns the position to which the servo was last commanded to move. Note that this method
     * does NOT read a position from the servo through any electrical means, as no such electrical
     * mechanism is, generally, available.
     *
     * @return the position to which the servo was last commanded to move, or Double.NaN
     * if no such position is known
     * @see #setPosition(double)
     * @see Double#NaN
     * @see Double#isNaN()
     */
    @Override
    public synchronized double getPosition() {
        double position = controller.getServoPosition(port);
        if (direction == Direction.REVERSE) position = reverse(position);
        return Mathf.clamp(Mathf.scale(position, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION), MIN_POSITION, MAX_POSITION);
    }

    /**
     * Scales the available movement range of the servo to be a subset of its maximum range. Subsequent
     * positioning calls will operate within that subset range. This is useful if your servo has
     * only a limited useful range of movement due to the physical hardware that it is manipulating
     * (as is often the case) but you don't want to have to manually scale and adjust the input
     * to {@link #setPosition(double) setPosition()} each time.
     *
     * <p>For example, if scaleRange(0.2, 0.8) is set; then servo positions will be
     * scaled to fit in that range:<br>
     * setPosition(0.0) scales to 0.2<br>
     * setPosition(1.0) scales to 0.8<br>
     * setPosition(0.5) scales to 0.5<br>
     * setPosition(0.25) scales to 0.35<br>
     * setPosition(0.75) scales to 0.65<br>
     * </p>
     *
     * <p>Note the parameters passed here are relative to the underlying full range of motion of
     * the servo, not its currently scaled range, if any. Thus, scaleRange(0.0, 1.0) will reset
     * the servo to its full range of movement.</p>
     *
     * @param min the lower limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @param max the upper limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @see #setPosition(double)
     */
    @Override
    public synchronized void scaleRange(double min, double max) {
        min = Mathf.clamp(min, MIN_POSITION, MAX_POSITION);
        max = Mathf.clamp(max, MIN_POSITION, MAX_POSITION);

        if (min >= max)
            throw new IllegalArgumentException("min must be less than max");

        limitPositionMin = min;
        limitPositionMax = max;
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return controller.getManufacturer();
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return deviceName;
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return controller.getConnectionInfo() + "; port " + port;
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return 1;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public synchronized void resetDeviceConfigurationForOpMode() {
        limitPositionMin = MIN_POSITION;
        limitPositionMax = MAX_POSITION;
        direction = Direction.FORWARD;
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        // no-op
    }

    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }
}
