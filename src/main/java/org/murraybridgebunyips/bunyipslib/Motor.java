package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.exception.OutOfRangeException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.PIDF;
import org.murraybridgebunyips.bunyipslib.external.PIDFFController;
import org.murraybridgebunyips.bunyipslib.external.SystemController;
import org.murraybridgebunyips.bunyipslib.external.ff.SimpleMotorFeedforward;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;

import java.util.ArrayList;
import java.util.Optional;

/**
 * Drop-in replacement for a {@link DcMotor} that uses custom control algorithms to operate {@link RunMode#RUN_USING_ENCODER}
 * and {@link RunMode#RUN_TO_POSITION} modes. Internally integrates a gain scheduler to allow for more precise
 * system coefficients against gravity and other external forces.
 * <p>
 * This class is designed to be down casted to a DcMotor where used, integrating with the current motor modes while
 * providing faster, predictable control systems.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class Motor extends DcMotorImplEx {
    private final ArrayList<InterpolatedLookupTable> rtpGains = new ArrayList<>();
    private final ArrayList<InterpolatedLookupTable> rueGains = new ArrayList<>();
    private final Encoder encoder;
    private double maxMagnitude = 1;
    private SystemController rtpController;
    private SystemController rueController;
    private Pair<Double, Double> rueInfo = null;
    private DcMotor.RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    /**
     * Wrap a DcMotor to use in the Motor class.
     *
     * @param motor the DcMotor from hardwareMap to use.
     */
    public Motor(DcMotor motor) {
        super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        // The actual motor should *always* be running in RUN_WITHOUT_ENCODER
        super.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = new Encoder(super::getCurrentPosition, super::getVelocity);
        setTargetPosition(getCurrentPosition());
    }

    /**
     * Call to use encoder overflow (exceeding 32767 ticks/sec) correction on {@link #getVelocity()}.
     */
    public void useEncoderOverflowCorrection() {
        encoder.useEncoderOverflowCorrection();
    }

    /**
     * @return the currently set RUN_TO_POSITION system controller
     */
    public Optional<SystemController> getRunToPositionController() {
        return Optional.ofNullable(rtpController);
    }

    /**
     * Set a system controller to use for {@link RunMode#RUN_TO_POSITION}.
     * <p>
     * The coefficients of this controller can be gain scheduled through {@link #scheduleRunToPositionGains()}.
     * Otherwise, you can adjust the coefficients directly on the controller instance and they will be respected, unless
     * a gain scheduler is set for this controller.
     * <p>
     * Note that when using a motor with this class, the PIDF coefficients attached to the motor itself will be used only
     * if a controller is not specified, and will only take a <b>snapshot at runtime</b> of these values to populate
     * a controller, making a fallback default PID controller to use. The SDK PIDF values are otherwise ignored.
     * Falling back on a default controller will also push a robot global warning as it is highly dangerous
     * to use the stock PIDF values in this context. Set your own controller using this method.
     *
     * @param controller the controller to use, recommended to use a closed-loop controller such as PID
     */
    public void setRunToPositionController(SystemController controller) {
        rtpController = controller;
        if (rtpController instanceof PIDF)
            ((PIDF) rtpController).getPIDFController().setTolerance(super.getTargetPositionTolerance());
    }

    /**
     * @return the currently set RUN_USING_ENCODER system controller
     */
    public Optional<SystemController> getRunUsingEncoderController() {
        return Optional.ofNullable(rueController);
    }

    /**
     * Set a system controller to use for {@link RunMode#RUN_USING_ENCODER}.
     * <p>
     * The coefficients of this controller can be gain scheduled through {@link #scheduleRunUsingEncoderGains()}.
     * Otherwise, you can adjust the coefficients directly on the controller instance and they will be respected, unless
     * a gain scheduler is set for this controller.
     * <p>
     * Note that when using a motor with this class, the PIDF coefficients attached to the motor itself will be used only
     * if a controller is not specified, and will only take a <b>snapshot at runtime</b> of these values to populate
     * a controller, making a fallback default VelocityFF controller to use. The SDK PIDF values are otherwise ignored.
     * Falling back on a default controller will also push a robot global warning as it is highly dangerous
     * to use the stock PIDF values in this context. Set your own controller using this method.
     *
     * @param controller                  the controller to use, recommended to use a PIDFF controller.
     * @param bufferFraction              fractional value for velocity control, must be in (0, 1].
     * @param maxAchievableTicksPerSecond your motor's spec for how many ticks/sec it can reach
     */
    public void setRunUsingEncoderController(SystemController controller, double bufferFraction, double maxAchievableTicksPerSecond) {
        if (bufferFraction <= 0 || bufferFraction > 1) {
            throw new OutOfRangeException(bufferFraction, 0, 1);
        }
        rueController = controller;
        rueInfo = new Pair<>(bufferFraction, maxAchievableTicksPerSecond);
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link RunMode#RUN_TO_POSITION} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous range of coefficients that will be used when {@link #setPower(double)} is called.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link RunMode#RUN_TO_POSITION} controller
     */
    public GainScheduling scheduleRunToPositionGains() {
        rtpGains.clear();
        return new GainScheduling(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link RunMode#RUN_USING_ENCODER} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous PID range that will be used when {@link #setPower(double)} is called.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link RunMode#RUN_USING_ENCODER} controller
     */
    public GainScheduling scheduleRunUsingEncoderGains() {
        rueGains.clear();
        return new GainScheduling(RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset the encoder value without stopping the motor. Will internally be called if the motor is attempted
     * to be set to {@link RunMode#STOP_AND_RESET_ENCODER}.
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * Get the encoder attached to this motor.
     *
     * @return the encoder object that is used for encoder readings on this motor
     */
    public Encoder getEncoder() {
        return encoder;
    }

    /**
     * @return the estimated acceleration of this motor. Ticks per second.
     */
    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    /**
     * @return the current position in ticks of this motor, with velocity estimation.
     */
    @Override
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the current velocity of this motor as specified by your settings. Ticks per second.
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Switches the motor to velocity control and tries to set the ticks/sec target.
     *
     * @param vel the desired ticks per second
     */
    @Override
    public void setVelocity(double vel) {
        // vel = buff * max * power
        // vel / (buff * max) = power
        if (rueInfo == null || rueInfo.first == null || rueInfo.second == null) {
            throw new IllegalStateException("RUN_USING_ENCODER controller not set up yet, cannot set velocity without setting the controller!");
        }
        setMode(RunMode.RUN_USING_ENCODER);
        setPower(vel / (rueInfo.first * rueInfo.second));
    }

    /**
     * Get the custom set mode for this Motor. Note that this will not reflect the actual SDK mode of the motor,
     * which is always set to {@link RunMode#RUN_WITHOUT_ENCODER}, but rather the equivalent mode this motor
     * is currently running in.
     */
    @Override
    public DcMotor.RunMode getMode() {
        return mode;
    }

    /**
     * Modified version of {@code setMode} where the modes will never actually be propagated to the motors, and instead
     * managed internally by the modified {@link #setPower(double)} method. The actual motor object will always be in
     * {@link RunMode#RUN_WITHOUT_ENCODER}.
     *
     * @param mode the new current run mode for this motor
     */
    @Override
    public void setMode(DcMotor.RunMode mode) {
        if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            setPower(0);
            resetEncoder();
            return;
        }
        this.mode = mode;
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     * @deprecated the RUN_TO_POSITION controller should be accessed directly coming from this Motor instance via {@link #getRunToPositionController()},
     * as this method context is not from a DcMotor downcast.
     */
    @Deprecated
    @Override
    public int getTargetPositionTolerance() {
        // Built-in support for PIDF as it is the most common use case for RTP
        if (rtpController != null && rtpController instanceof PIDF) {
            return (int) Math.ceil(((PIDF) rtpController).getPIDFController().getTolerance()[0]);
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     * @deprecated the RUN_TO_POSITION controller should be accessed directly coming from this Motor instance via {@link #getRunToPositionController()},
     * as this method context is not from a DcMotor downcast.
     */
    @Deprecated
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (rtpController != null && rtpController instanceof PIDF) {
            ((PIDF) rtpController).getPIDFController().setTolerance(tolerance);
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Note that this method will try to access the {@link RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PID controller.
     *
     * @inheritDoc
     * @deprecated the RUN_TO_POSITION controller should be accessed directly coming from this Motor instance via {@link #getRunToPositionController()},
     * as this method context is not from a DcMotor downcast.
     */
    @Deprecated
    @Override
    public boolean isBusy() {
        if (rtpController != null && rtpController instanceof PIDF) {
            return mode == RunMode.RUN_TO_POSITION && !((PIDF) rtpController).getPIDFController().atSetPoint();
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Switches the motor to velocity control and tries to set the angular velocity of the motor based on the intrinsic
     * {@link MotorConfigurationType} configuration.
     *
     * @param vel  the desired angular rate, in units per second
     * @param unit the units in which angularRate is expressed
     */
    @Override
    public void setVelocity(double vel, AngleUnit unit) {
        double tpr = motorType.getTicksPerRev();
        if (tpr <= 0) {
            throw new IllegalStateException(formatString("The Ticks Per Revolution attribute has not been set for this motor (% on port %). You will have to clone the current motorType, set the ticksPerRev, and set the new motorType to the cloned copy.", getDeviceName(), getPortNumber()));
        }
        double radsPerSec = UnnormalizedAngleUnit.RADIANS.fromUnit(unit.getUnnormalized(), vel);
        // Will assume no reduction, the user can scale the velocity on their own terms
        setVelocity(EncoderTicks.fromAngle(Radians.of(radsPerSec), (int) tpr, 1));
    }

    /**
     * Sets the maximum power magnitude (applies for both negative and positive powers) this motor can run at.
     * This will ensure all calls to {@link #setPower} will never exceed the maximum boundary as defined by this function.
     * <p>
     * Note: Further calls to {@link #setPower} that exceed the new domain will be scaled (for example, if the max power
     * was defined as 0.8, {@code setPower(1)} will set the motor power to 0.8 in {@code RUN_WITHOUT_ENCODER} mode)
     *
     * @param magnitude maximum absolute magnitude of {@link #setPower}, default of 1.0 (SDK), applies bidirectionally
     */
    public void setMaxPower(double magnitude) {
        maxMagnitude = Math.min(1, Math.abs(magnitude));
    }

    /**
     * Update system controllers and propagate new power levels to the motor.
     * <p>
     * Note: <b>This method needs to be called periodically (as part of the active loop)
     * in order to update the system controllers that respond to dynamic conditions.</b>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0];
     *              this value is scaled by {@link #setMaxPower}, if used.
     */
    @Override
    public void setPower(double power) {
        double magnitude = 0;
        switch (mode) {
            case RUN_TO_POSITION:
                if (rtpController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(RunMode.RUN_TO_POSITION);
                    String msg = formatString("[% on port %] No RUN_TO_POSITION controller was specified. This motor will be using the default PIDF coefficients to create a fallback PIDF controller with values from %. You must set your own controller through setRunToPositionController().", getDeviceName(), getPortNumber(), coeffs);
                    Dbg.error(msg);
                    RobotLog.addGlobalWarningMessage(msg);
                    rtpController = new PIDFController(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
                    ((PIDFController) rtpController).setTolerance(super.getTargetPositionTolerance());
                }
                if (!rtpGains.isEmpty())
                    rtpController.setCoefficients(rtpGains.stream().mapToDouble(this::getClampedInterpolatedGain).toArray());
                // In a RUN_TO_POSITION context, the controller is used for error correction, which will multiply the
                // allowed power by the user against the encoder error by your (usually PID) controller.
                magnitude = Math.abs(power) * rtpController.calculate(getCurrentPosition(), getTargetPosition());
                break;
            case RUN_USING_ENCODER:
                if (rueController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
                    String msg = formatString("[% on port %] No RUN_USING_ENCODER controller was specified. This motor will be using the default PIDF coefficients to create a fallback PID and static FF controller with values from %. You must set your own controller through setRunUsingEncoderController().", getDeviceName(), getPortNumber(), coeffs);
                    Dbg.error(msg);
                    RobotLog.addGlobalWarningMessage(msg);
                    PIDController pid = new PIDController(coeffs.p, coeffs.i, coeffs.d);
                    rueController = new PIDFFController(pid, new SimpleMotorFeedforward(coeffs.f, 0, 0), encoder);
                    rueInfo = new Pair<>(1.0, motorType.getAchieveableMaxTicksPerSecond());
                }
                if (!rueGains.isEmpty())
                    rueController.setCoefficients(rueGains.stream().mapToDouble(this::getClampedInterpolatedGain).toArray());
                if (rueInfo == null || rueInfo.first == null || rueInfo.second == null)
                    throw new EmergencyStop("Invalid motor information configuration passed for RUN_USING_ENCODER.");
                // In RUN_USING_ENCODER, the controller is expected to take in the current velocity and target velocity,
                // which usually consists internally of a PID and feedforward controller.
                if (power == 0) {
                    // We need an immediate stop if there's no power/velo requested
                    magnitude = 0;
                    break;
                }
                double targetVel = rueInfo.first * rueInfo.second * power;
                double output = rueController instanceof PIDFFController
                        ? ((PIDFFController) rueController).calculateVelo(getVelocity(), targetVel)
                        : rueController.calculate(getVelocity(), targetVel);
                magnitude = output / rueInfo.second;
                break;
            case RUN_WITHOUT_ENCODER:
                magnitude = power;
                break;
        }
        // Clamp and rescale depending on the maximum magnitude
        magnitude = Mathf.clamp(magnitude, -1, 1);
        super.setPower(Mathf.scale(Mathf.clamp(magnitude, -1, 1), -1, 1, -maxMagnitude, maxMagnitude));
    }

    private double getClampedInterpolatedGain(InterpolatedLookupTable lut) {
        int curr = getCurrentPosition();
        int res = lut.testOutOfRange(curr);
        if (res == 0) return lut.get(curr);
        return res == -1 ? lut.getMin() : lut.getMax();
    }

    /**
     * Gain scheduler builder which will interpolate on build and populate gains. Not to be used as-is.
     *
     * @see #scheduleRunToPositionGains()
     * @see #scheduleRunUsingEncoderGains()
     */
    public class GainScheduling {
        private final ArrayList<InterpolatedLookupTable> gains;

        private GainScheduling(DcMotor.RunMode targetMode) {
            gains = targetMode == DcMotor.RunMode.RUN_TO_POSITION ? rtpGains : rueGains;
        }

        /**
         * Specify a position in encoder ticks that you want the gains of your system controller to be.
         * This should be used with known values, such as knowing for 0 ticks the gains should be 1,0,0, etc.
         * <p>
         * The more positions that are added to this builder, the more accurate your final gain scheduling model will be.
         * <p>
         * Note that the maximum domain that interpolated gains scheduling will be available for is limited by the min/max values
         * you supply in this builder. Your controller will clamp to the boundary if the encoder is out of the function domain.
         * <p>
         * Multiple calls to construct this builder will discard old gain scheduling.
         *
         * @param positionTicks the position in encoder ticks that the system controller coefficients should be
         * @param coeffs        the coefficients at encoder ticks position
         * @return the builder
         */
        public GainScheduling atPosition(double positionTicks, double... coeffs) {
            for (int i = 0; i < coeffs.length; i++) {
                if (gains.size() <= i) {
                    gains.add(new InterpolatedLookupTable());
                }
                gains.get(i).add(positionTicks, coeffs[i]);
            }
            return this;
        }

        /**
         * Create the interpolated lookup tables for use in gains scheduling.
         */
        public void build() {
            for (int i = 0; i < gains.size(); i++) {
                gains.get(i).createLUT();
            }
        }
    }
}
