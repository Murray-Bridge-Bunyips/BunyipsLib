package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Volts;

import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.exception.OutOfRangeException;
import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.EmergencyStop;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Encoder;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.EncoderTicks;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.InterpolatedLookupTable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.SimpleMotorFeedforward;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Voltage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Drop-in replacement for a {@link DcMotor} that uses custom control algorithms to operate {@link DcMotor.RunMode#RUN_USING_ENCODER}
 * and {@link DcMotor.RunMode#RUN_TO_POSITION} modes. Internally integrates a gain scheduler to allow for more precise
 * system coefficients against gravity and other external forces. This class effectively wraps an entire DcMotor and
 * regulates all of the operations.
 * <p>
 * This class is designed to be down casted to a DcMotor where used, integrating with the current motor modes while
 * providing faster, predictable control systems.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
@SuppressWarnings("deprecation")
public class Motor extends SimpleRotator implements DcMotorEx {
    /**
     * The encoder object that is used for position and velocity readings on this motor.
     * <p>
     * To reverse the directionality of the encoder, but not the output power, you can access this object
     * and call the {@code setDirection} method on it. Updating the general direction of the motor will update both.
     */
    public final Encoder encoder;

    protected final DcMotorControllerEx controller;
    protected final int port;

    private final ArrayList<InterpolatedLookupTable> rtpGains = new ArrayList<>();
    private final ArrayList<InterpolatedLookupTable> rueGains = new ArrayList<>();

    private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private SystemController rtpController;
    private SystemController rueController;
    private Pair<Double, Double> rueInfo = null;
    private VoltageSensor nominalVoltageSensor = null;
    private double nominalVoltage = 12.0;

    private int rawTargetPosition = 0;

    /**
     * Wrap a DcMotor to use in the Motor class.
     *
     * @param motor the DcMotor from hardwareMap to use.
     */
    public Motor(@NonNull DcMotor motor) {
        super(motor);
        // Take control over this motor's controller, we don't need to manage the motor configuration as that
        // should be on the controller and has been detached in the overhead DcMotor
        controller = (DcMotorControllerEx) motor.getController();
        port = motor.getPortNumber();
        // The actual motor should *always* be running in RUN_WITHOUT_ENCODER
        synchronized (controller) {
            controller.setMotorMode(port, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        DcMotorEx dme = (DcMotorEx) motor;
        encoder = new Encoder(motor::getCurrentPosition, dme::getVelocity);
        rawTargetPosition = getCurrentPosition();
    }

    /**
     * Utility for debugging an encoder on telemetry with current encoder/power information.
     *
     * @param motor     The motor to debug
     * @param name      The name to debug as
     * @param telemetry The telemetry to debug to
     */
    public static void debug(@NonNull DcMotor motor, @NonNull String name, @NonNull Telemetry telemetry) {
        telemetry.addData(Text.format("% Position (t, port %)", name, motor.getPortNumber()), motor.getCurrentPosition());
        telemetry.addData(Text.format("% Target (t, port %)", name, motor.getPortNumber()), motor.getTargetPosition());
        telemetry.addData(Text.format("% Power (port %)", name, motor.getPortNumber()), motor.getPower());
        if (motor instanceof DcMotorEx) {
            telemetry.addData(Text.format("% Velocity (t/s, port %)", name, motor.getPortNumber()), ((DcMotorEx) motor).getVelocity());
            telemetry.addData(Text.format("% Current (A, port %)", name, motor.getPortNumber()), ((DcMotorEx) motor).getCurrent(CurrentUnit.AMPS));
        }
    }

    // hello i am Giulio

    /**
     * Sets a voltage sensor to use for nominal power calculation.
     * See {@link #setNominalVoltage(Measure)} to set a nominal voltage directly, defaults to 12V.
     *
     * @param nominalVoltageSensorMapping the sensor mapping to use for nominal voltage calculation, usually {@code hardwareMap.voltageSensor}
     */
    public void setNominalVoltageSensor(@NonNull HardwareMap.DeviceMapping<VoltageSensor> nominalVoltageSensorMapping) {
        nominalVoltageSensor = nominalVoltageSensorMapping.iterator().next();
    }

    /**
     * Sets the nominal voltage to use for power calculations.
     * The equation used is {@code applied = power * (nominalVoltage / sensorVoltage)}.
     * <p>
     * A voltage sensor <b>must</b> be set for this to work via {@link #setNominalVoltageSensor(HardwareMap.DeviceMapping)}
     * This value will be ignored if no sensor is set.
     *
     * @param nominalVoltage the nominal voltage parameter to use, defaults to 12V
     */
    public void setNominalVoltage(@NonNull Measure<Voltage> nominalVoltage) {
        this.nominalVoltage = Math.abs(nominalVoltage.in(Volts));
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
    @Nullable
    public SystemController getRunToPositionController() {
        return rtpController;
    }

    /**
     * Set a system controller to use for {@link DcMotor.RunMode#RUN_TO_POSITION}.
     * <p>
     * The coefficients of this controller can be gain scheduled through {@link #scheduleRunToPositionGains()}.
     * Otherwise, you can adjust the coefficients directly on the controller instance and they will be respected, unless
     * a gain scheduler is set for this controller.
     * <p>
     * Note that when using a motor with this class, the PIDF coefficients attached to the motor itself will be used only
     * if a controller is not specified, and will only take a <b>snapshot at runtime</b> of these values to populate
     * a controller, making a fallback default PID controller to use. The SDK PIDF values are otherwise ignored, and
     * the methods to set and get these coefficients as part of the {@link DcMotor} interface have been overridden.
     * Falling back on a default controller will also push a robot global warning as it is highly dangerous
     * to use the stock PIDF values in this context. Set your own controller using this method.
     *
     * @param controller the controller to use, recommended to use a closed-loop controller such as PID
     */
    public void setRunToPositionController(@Nullable SystemController controller) {
        if (controller == null)
            return;
        rtpController = controller;
        if (rtpController.pidf().isPresent())
            rtpController.pidf().get().setTolerance(LynxConstants.DEFAULT_TARGET_POSITION_TOLERANCE);
    }

    /**
     * @return the currently set RUN_USING_ENCODER system controller
     */
    @Nullable
    public SystemController getRunUsingEncoderController() {
        return rueController;
    }

    /**
     * Set a system controller to use for {@link DcMotor.RunMode#RUN_USING_ENCODER}.
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
     * @param controller the controller to use, recommended to use a PIDFF controller.
     */
    public void setRunUsingEncoderController(@Nullable SystemController controller) {
        setRunUsingEncoderController(1, getMotorType().getAchieveableMaxTicksPerSecond(), controller);
    }

    /**
     * Set a system controller to use for {@link DcMotor.RunMode#RUN_USING_ENCODER}.
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
     * @param bufferFraction              fractional value for velocity control, must be in (0, 1].
     * @param maxAchievableTicksPerSecond your motor's spec for how many ticks/sec it can reach
     * @param controller                  the controller to use, recommended to use a PIDFF controller.
     */
    public void setRunUsingEncoderController(double bufferFraction, double maxAchievableTicksPerSecond, @Nullable SystemController controller) {
        if (controller == null)
            return;
        if (bufferFraction <= 0 || bufferFraction > 1) {
            throw new OutOfRangeException(LocalizedFormats.OUT_OF_RANGE_LEFT, bufferFraction, 0, 1);
        }
        rueController = controller;
        rueInfo = new Pair<>(bufferFraction, maxAchievableTicksPerSecond);
    }

    /**
     * Set a system controller to use for {@link DcMotor.RunMode#RUN_USING_ENCODER}.
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
     * @param bufferFraction fractional value for velocity control, must be in (0, 1].
     * @param controller     the controller to use, recommended to use a PIDFF controller.
     */
    public void setRunUsingEncoderController(double bufferFraction, @NonNull SystemController controller) {
        if (bufferFraction <= 0 || bufferFraction > 1) {
            throw new OutOfRangeException(LocalizedFormats.OUT_OF_RANGE_LEFT, bufferFraction, 0, 1);
        }
        rueController = controller;
        rueInfo = new Pair<>(bufferFraction, getMotorType().getAchieveableMaxTicksPerSecond());
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link DcMotor.RunMode#RUN_TO_POSITION} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous range of coefficients that will be used when {@link #setPower(double)} is called.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link DcMotor.RunMode#RUN_TO_POSITION} controller
     */
    @NonNull
    public GainScheduling scheduleRunToPositionGains() {
        rtpGains.clear();
        return new GainScheduling(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Call to build a list of encoder tick positions where you want your {@link DcMotor.RunMode#RUN_USING_ENCODER} system controller
     * gains to be. When this builder is built with {@code build()}, it will interpolate between each value to provide a
     * continuous PID range that will be used when {@link #setPower(double)} is called.
     *
     * @return a builder to specify encoder tick positions to gains of your {@link DcMotor.RunMode#RUN_USING_ENCODER} controller
     */
    @NonNull
    public GainScheduling scheduleRunUsingEncoderGains() {
        rueGains.clear();
        return new GainScheduling(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset the encoder value without stopping the motor. Will internally be called if the motor is attempted
     * to be set to {@link DcMotor.RunMode#STOP_AND_RESET_ENCODER}. The target position will also be reset to 0.
     */
    public synchronized void resetEncoder() {
        encoder.reset();
        setTargetPosition(0);
    }

    /**
     * @return the estimated acceleration of this motor. Ticks per second.
     */
    public synchronized double getAcceleration() {
        return encoder.getAcceleration();
    }

    /**
     * @return the current position in ticks of this motor, while performing velocity estimation.
     */
    @Override
    public synchronized int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the current velocity of this motor as specified by your settings, while performing acceleration estimation. Return ticks per second.
     */
    @Override
    public synchronized double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Switches the motor to velocity control and tries to set the ticks/sec target.
     *
     * @param vel the desired ticks per second
     */
    @Override
    public synchronized void setVelocity(double vel) {
        // vel = buff * max * power
        // vel / (buff * max) = power
        if (rueInfo == null || rueInfo.first == null || rueInfo.second == null) {
            throw new IllegalStateException("RUN_USING_ENCODER controller not set up yet, cannot set velocity without setting the controller!");
        }
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(vel / (rueInfo.first * rueInfo.second));
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see #setVelocity(double, AngleUnit)
     */
    @Override
    public synchronized double getVelocity(@NonNull AngleUnit unit) {
        Measure<Angle> vel = EncoderTicks.toAngle((int) getVelocity(), (int) getMotorType().getTicksPerRev(), 1);
        return unit == AngleUnit.DEGREES ? vel.in(Degrees) : vel.in(Radians);
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note this will construct a new PID controller for use in the {@code rtp} or {@code rue} RunModes as per
     * the Motor class.
     *
     * @param mode            either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see #getPIDCoefficients(DcMotor.RunMode)
     */
    @Override
    public void setPIDCoefficients(@NonNull RunMode mode, @NonNull PIDCoefficients pidCoefficients) {
        mode = mode.migrate();
        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            setRunToPositionController(new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d));
        } else if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            setRunUsingEncoderController(1, getMotorType().getAchieveableMaxTicksPerSecond(), new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d));
        }
    }

    /**
     * This method is a superset enhancement to {@link #setPIDCoefficients}.
     * Note this will construct a new PIDF controller for use in the {@code rtp} or {@code rue} RunModes as per
     * the Motor class, or set an existing PIDF controller coefficients if it already exists.
     *
     * @param mode             either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @param pidfCoefficients the new coefficients to use when in that mode on this motor
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(DcMotor.RunMode)
     */
    @Override
    public void setPIDFCoefficients(@NonNull RunMode mode, @NonNull PIDFCoefficients pidfCoefficients) {
        mode = mode.migrate();
        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (rtpController == null) {
                setRunToPositionController(new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f));
            } else if (rtpController.pidf().isPresent()) {
                rtpController.pidf().get().setPIDF(pidfCoefficients);
            } else {
                throw new UnsupportedOperationException("Can't access information on the currently used RTP controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, which makes this method incapable of setting these coefficients.");
            }
        } else if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            if (rueController == null) {
                setRunUsingEncoderController(1, getMotorType().getAchieveableMaxTicksPerSecond(), new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f));
            } else if (rueController.pidf().isPresent()) {
                rueController.pidf().get().setPIDF(pidfCoefficients);
            } else {
                throw new UnsupportedOperationException("Can't access information on the currently used RUE controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, which makes this method incapable of setting these coefficients.");
            }
        }
    }

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotor.RunMode#RUN_USING_ENCODER}
     * mode. Note this will either set a new controller if one is not defined, or try to set the PIDF coefficients
     * on the current controller to these coefficients.
     *
     * @param p proportional
     * @param i integral
     * @param d derivative
     * @param f feedforward
     * @see #setPIDFCoefficients(DcMotor.RunMode, PIDFCoefficients)
     */
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (rueController == null) {
            setRunUsingEncoderController(1, getMotorType().getAchieveableMaxTicksPerSecond(), new PIDFController(p, i, d, f));
        } else if (rueController.pidf().isPresent()) {
            rueController.pidf().get().setPIDF(p, i, d, f);
        } else {
            throw new UnsupportedOperationException("Can't access information on the currently used RUE controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, which makes this method incapable of setting these coefficients.");
        }
    }

    /**
     * A shorthand for setting the P coefficient for the {@link DcMotor.RunMode#RUN_TO_POSITION}
     * mode. Note this will either set a new controller if one is not defined, or try to set the PIDF coefficients
     * on the current controller with this P coefficient in place of the old one. Other coefficients will be preserved.
     *
     * @param p proportional
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(DcMotor.RunMode, PIDFCoefficients)
     */
    @Override
    public void setPositionPIDFCoefficients(double p) {
        if (rtpController == null) {
            setRunToPositionController(new PController(p));
        } else if (rtpController.pidf().isPresent()) {
            PIDFController controller = rtpController.pidf().get();
            double[] coeffs = controller.getCoefficients();
            controller.setPIDF(p, coeffs[1], coeffs[2], coeffs[3]);
        } else {
            throw new UnsupportedOperationException("Can't access information on the currently used RTP controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, which makes this method incapable of setting these coefficients.");
        }
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     */
    @NonNull
    @Override
    public PIDCoefficients getPIDCoefficients(@NonNull DcMotor.RunMode mode) {
        PIDFCoefficients coeffs = getPIDFCoefficients(mode);
        return new PIDCoefficients(coeffs.p, coeffs.i, coeffs.d);
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see #setPIDFCoefficients(DcMotor.RunMode, PIDFCoefficients)
     */
    @NonNull
    @Override
    public PIDFCoefficients getPIDFCoefficients(@NonNull DcMotor.RunMode mode) {
        double[] coeffs = new double[0];
        mode = mode.migrate();
        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (rtpController == null || rtpController.pidf().isEmpty()) {
                throw new UnsupportedOperationException("Can't access information on the currently used RTP controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, or does not exist, which makes this method incapable of getting these coefficients.");
            }
            coeffs = rtpController.pidf().get().getCoefficients();
        } else if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            if (rueController == null || rueController.pidf().isEmpty()) {
                throw new UnsupportedOperationException("Can't access information on the currently used RUE controller. This is because the currently set controller is not a PIDF or PIDF-derived controller, or does not exist, which makes this method incapable of getting these coefficients.");
            }
            coeffs = rueController.pidf().get().getCoefficients();
        }
        if (coeffs.length == 0) {
            throw new IllegalArgumentException("Invalid runmode");
        }
        return new PIDFCoefficients(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    }

    /**
     * Individually energizes this particular motor
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorEnable() {
        controller.setMotorEnable(port);
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorDisable() {
        controller.setMotorDisable(port);
    }

    /**
     * Returns whether this motor is energized
     *
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    @Override
    public boolean isMotorEnabled() {
        return controller.isMotorEnabled(port);
    }

    /**
     * Get the custom set mode for this Motor. Note that this will not reflect the actual SDK mode of the motor,
     * which is always set to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}, but rather the equivalent mode this motor
     * is currently running in.
     */
    @NonNull
    @Override
    public DcMotor.RunMode getMode() {
        return mode;
    }

    /**
     * Modified version of {@code setMode} where the modes will never actually be propagated to the motors, and instead
     * managed internally by the modified {@link #setPower(double)} method. The actual motor object will always be in
     * {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}.
     *
     * @param mode the new current run mode for this motor
     */
    @Override
    public synchronized void setMode(@NonNull DcMotor.RunMode mode) {
        if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            setPower(0);
            resetEncoder();
            return;
        }
        this.mode = mode;
    }

    /**
     * Note that this method will try to access the {@link DcMotor.RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PIDF controller.
     *
     * @inheritDoc
     * @deprecated the RUN_TO_POSITION controller should be accessed directly coming from this Motor instance via {@link #getRunToPositionController()},
     * as this method context is not from a DcMotor downcast.
     */
    @Deprecated
    @Override
    public int getTargetPositionTolerance() {
        // Built-in support for PIDF as it is the most common use case for RTP,
        // the Motor class does not manage target position tolerance at all
        if (rtpController != null && rtpController.pidf().isPresent()) {
            return (int) Math.ceil(rtpController.pidf().get().getTolerance()[0]);
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Note that this method will try to access the {@link DcMotor.RunMode#RUN_TO_POSITION} controller to access information there. This
     * is otherwise unsupported and you should try to access the actual controller to see this information, unless
     * you are downcasting in which this will assume you are using a PIDF controller.
     *
     * @inheritDoc
     * @deprecated the RUN_TO_POSITION controller should be accessed directly coming from this Motor instance via {@link #getRunToPositionController()},
     * as this method context is not from a DcMotor downcast.
     */
    @Deprecated
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (rtpController != null && rtpController.pidf().isPresent()) {
            rtpController.pidf().get().setTolerance(tolerance);
            return;
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Returns the current consumed by this motor.
     *
     * @param unit current units
     * @return the current consumed by this motor.
     */
    @Override
    public double getCurrent(@NonNull CurrentUnit unit) {
        return controller.getMotorCurrent(port, unit);
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    @Override
    public double getCurrentAlert(@NonNull CurrentUnit unit) {
        return controller.getMotorCurrentAlert(port, unit);
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit    current units
     */
    @Override
    public void setCurrentAlert(double current, @NonNull CurrentUnit unit) {
        controller.setMotorCurrentAlert(port, current, unit);
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     *
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    @Override
    public boolean isOverCurrent() {
        return controller.isMotorOverCurrent(port);
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    @NonNull
    @Override
    public MotorConfigurationType getMotorType() {
        return controller.getMotorType(port);
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(@NonNull MotorConfigurationType motorType) {
        controller.setMotorType(port, motorType);
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    @NonNull
    @Override
    public DcMotorController getController() {
        return controller;
    }

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     *
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see #getController()
     */
    @Override
    public int getPortNumber() {
        return port;
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @NonNull
    @Override
    public synchronized ZeroPowerBehavior getZeroPowerBehavior() {
        return controller.getMotorZeroPowerBehavior(port);
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public synchronized void setZeroPowerBehavior(@NonNull ZeroPowerBehavior zeroPowerBehavior) {
        controller.setMotorZeroPowerBehavior(port, zeroPowerBehavior);
    }

    /**
     * Sets the zero power behavior of the motor to {@link ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     *     motor.setPowerFloat();
     *     motor.setPower(0.0);
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     * {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
     * {@link #setPower(double) setPower()}.
     */
    @Deprecated
    @Override
    public synchronized void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    @Override
    public synchronized boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    @Override
    public synchronized int getTargetPosition() {
        // May as well let the motor controller manage target position, there is nothing interfering with doing so
        return rawTargetPosition * (getDirection() == Direction.FORWARD ? 1 : -1);
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true if using a PIDF controller.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link DcMotor.RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     * <p>
     * Additionally note that the target positioning functionality of the Motor class handles
     * the propagation of target position following internally.
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(DcMotor.RunMode)
     * @see DcMotor.RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public synchronized void setTargetPosition(int position) {
        rawTargetPosition = position * (getDirection() == Direction.FORWARD ? 1 : -1);
    }

    /**
     * Note that this method will try to access the {@link DcMotor.RunMode#RUN_TO_POSITION} controller to access information there. This
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
        if (rtpController != null && rtpController.pidf().isPresent()) {
            return mode == DcMotor.RunMode.RUN_TO_POSITION && !rtpController.pidf().get().atSetPoint();
        }
        throw new UnsupportedOperationException("Can't access target position information on the currently used RTP controller. It may be the case that this controller is open-loop, or not a PID controller, as any tolerance configuration should be modified by your controller, not by this method.");
    }

    /**
     * Switches the motor to velocity control and tries to set the angular velocity of the motor based on the intrinsic
     * {@link MotorConfigurationType} configuration.
     *
     * @param angVel the desired angular rate, in units per second
     * @param unit   the units in which angVel is expressed
     */
    @Override
    public synchronized void setVelocity(double angVel, @NonNull AngleUnit unit) {
        double tpr = getMotorType().getTicksPerRev();
        if (tpr <= 0) {
            throw new IllegalStateException(Text.format("The Ticks Per Revolution attribute has not been set for this motor (port %). You will have to clone the current motorType, set the ticksPerRev, and set the new motorType to the cloned copy.", port));
        }
        double radsPerSec = UnnormalizedAngleUnit.RADIANS.fromUnit(unit.getUnnormalized(), angVel);
        // Will assume no reduction, the user can scale the velocity on their own terms
        setVelocity(EncoderTicks.fromAngle(Radians.of(radsPerSec), (int) tpr, 1));
    }

    /**
     * Update system controllers and propagate new power levels to the motor.
     * <p>
     * Note: <b>This method must be called periodically (as part of the active loop)
     * in order to update the system controllers and timers that respond to dynamic conditions.</b>
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0];
     *              this value is scaled by {@link #setMaxPower}, if used.
     */
    @Override
    public synchronized void setPower(double power) {
        double magnitude = 0;
        switch (mode) {
            case RUN_TO_POSITION:
                if (rtpController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    String msg = Text.format("[Port %] No RUN_TO_POSITION controller was specified. This motor will be using the default PIDF coefficients to create a fallback PIDF controller with values from %. You must set your own controller through setRunToPositionController().", port, coeffs);
                    Dbg.error(msg);
                    RobotLog.addGlobalWarningMessage(msg);
                    rtpController = new PIDFController(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
                    ((PIDFController) rtpController).setTolerance(LynxConstants.DEFAULT_TARGET_POSITION_TOLERANCE);
                }
                if (!rtpGains.isEmpty())
                    rtpController.setCoefficients(rtpGains.stream().mapToDouble(this::getClampedInterpolatedGain).toArray());
                // In a RUN_TO_POSITION context, the controller is used for error correction, which will multiply the
                // allowed power by the user against the encoder error by your (usually PID) controller.
                magnitude = Math.abs(power) * Mathf.clamp(rtpController.calculate(getCurrentPosition(), getTargetPosition()), -1, 1);
                break;
            case RUN_USING_ENCODER:
                if (rueController == null) {
                    PIDFCoefficients coeffs = getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                    String msg = Text.format("[Port %] No RUN_USING_ENCODER controller was specified. This motor will be using the default PIDF coefficients to create a fallback PID and static FF controller with values from %. You must set your own controller through setRunUsingEncoderController().", port, coeffs);
                    Dbg.error(msg);
                    RobotLog.addGlobalWarningMessage(msg);
                    PIDController pid = new PIDController(coeffs.p, coeffs.i, coeffs.d);
                    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(coeffs.f, 0, 0, encoder::getVelocity, encoder::getAcceleration);
                    rueController = pid.compose(ff, Double::sum);
                    rueInfo = new Pair<>(1.0, getMotorType().getAchieveableMaxTicksPerSecond());
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
                double output = rueController.calculate(getVelocity(), targetVel);
                magnitude = output / rueInfo.second;
                break;
            case RUN_WITHOUT_ENCODER:
                magnitude = power;
                break;
        }
        double mul = 1;
        if (nominalVoltageSensor != null)
            mul = nominalVoltage / nominalVoltageSensor.getVoltage();
        // Clamping, rescaling, and performance optimisations occur in SimpleRotator
        super.setPower(magnitude * mul);
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
        @NonNull
        public GainScheduling atPosition(double positionTicks, @NonNull double... coeffs) {
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
            for (InterpolatedLookupTable lut : gains) {
                lut.createLUT();
            }
        }
    }
}
