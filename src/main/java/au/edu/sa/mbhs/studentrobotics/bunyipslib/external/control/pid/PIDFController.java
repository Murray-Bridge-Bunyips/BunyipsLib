package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Optional;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.CompositeController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Filter;

/**
 * This is a <a href="https://en.wikipedia.org/wiki/PID_controller">PID controller</a> for your robot.
 * Internally, it performs all the calculations for you. This controller also includes a feedforward, {@code kF}, which
 * is multiplied by the setpoint. Additional features such as output clamping, integral clamping, integral zone,
 * continuous input, derivative smoothening, and auto-integral clearing are also included.
 * <p>
 * You need to tune your values to the appropriate amounts in order to properly utilise these calculations.
 * <p>
 * The equation we will use is:
 * {@code u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF * setPoint}
 * where {@code e(t) = r(t) - y(t)} and {@code r(t)} is the setpoint and {@code y(t)} is the
 * measured value. If we consider {@code e(t)} the positional error, then
 * {@code int(0,t)[e(t')dt']} is the total error and {@code e'(t)} is the derivative error.
 * <a href="https://github.com/FTCLib/FTCLib/blob/cedc52cee1bb549324c1ca5059c5feec3c054902/core/src/main/java/com/arcrobotics/ftclib/controller/PIDFController.java">Source</a>
 * <p>
 * This controller can be combined with a feedforward controller to create a {@link CompositeController}, via
 * the {@link SystemController} interface.
 *
 * @since 1.0.0-pre
 */
public class PIDFController implements SystemController {
    /**
     * Proportional gain.
     */
    public double kP;
    /**
     * Integral gain.
     */
    public double kI;
    /**
     * Derivative gain.
     */
    public double kD;
    /**
     * Feedforward setpoint gain.
     */
    public double kF;
    /**
     * Whether to clear the integral term when a new setpoint is set.
     */
    public boolean clearIOnNewSetpoint;
    /**
     * The minimum applied value for the integral term.
     */
    public double minIClamp;
    /**
     * The maximum applied value for the integral term.
     */
    public double maxIClamp;
    /**
     * The zone around the error where the integral term is active.
     */
    public double iZone = Double.POSITIVE_INFINITY;
    /**
     * The lower clamp for the output.
     */
    public double lowerClamp = -Double.MAX_VALUE;
    /**
     * The upper clamp for the output.
     */
    public double upperClamp = Double.MAX_VALUE;
    /**
     * The positional error tolerance.
     */
    public double errorToleranceP = 0.05;
    /**
     * The derivative/velocity error tolerance.
     */
    public double errorToleranceD = Double.POSITIVE_INFINITY;

    protected boolean continuousInput;
    protected double minContinuousInput, maxContinuousInput;

    private double setPoint;
    private double currentPv;

    private Filter.LowPass derivativeFilter = new Filter.LowPass(0.01);

    private double errorP;
    private double errorI;
    private double errorD;

    private boolean hasMeasured;
    private double prevErrorP;
    private double lastTimeStamp;
    private double period;

    /**
     * The base constructor for the PIDF controller.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     */
    public PIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    /**
     * This is the full constructor for the PIDF controller. This controller
     * also includes a feedforward value which is useful for fighting friction.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     * @param sp The setpoint of the pid control loop.
     * @param pv The measured value of the pid control loop. We want sp = pv, or to the degree
     *           such that sp - pv, or e(t) is less than the tolerance.
     */
    public PIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sp;
        currentPv = pv;

        minIClamp = -1.0;
        maxIClamp = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorP = setPoint - currentPv;
        reset();
    }

    /**
     * Clamps the maximum output magnitude that can be achieved via {@link #calculate()}.
     *
     * @param lower the lower clamp
     * @param upper the upper clamp
     * @return this
     */
    @NonNull
    public PIDFController setOutputClamps(double lower, double upper) {
        lowerClamp = lower;
        upperClamp = upper;
        return this;
    }

    /**
     * Sets whether the integral term should be cleared when a new setpoint is set.
     *
     * @param clearIOnNewSetpoint Whether to clear the integral term.
     * @return this
     */
    @NonNull
    public PIDFController setClearIntegralOnNewSetpoint(boolean clearIOnNewSetpoint) {
        this.clearIOnNewSetpoint = clearIOnNewSetpoint;
        return this;
    }

    /**
     * Set the bounds for the integral term.
     * <p>
     * The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param integralMin The minimum value for the integral term. Defaults to -1.0
     * @param integralMax The maximum value for the integral term. Defaults to 1.0
     * @return this
     */
    @NonNull
    public PIDFController setIntegrationBounds(double integralMin, double integralMax) {
        minIClamp = integralMin;
        maxIClamp = integralMax;
        return this;
    }

    /**
     * Sets the Integration zone range.
     * <p>
     * When the absolute value of the position error is greater than the integration zone,
     * the total accumulated error will reset to zero, disabling integral gain until the absolute value of
     * the position error is less than the integration zone. This is used to prevent integral windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables integration zone functionality.
     *
     * @param iZone the zone where the integral term is active, where if the error is within this zone, the integral term is active
     * @return this
     */
    public PIDFController setIntegrationZone(double iZone) {
        this.iZone = iZone;
        return this;
    }

    /**
     * Enables continuous input.
     * <p>
     * Rather than using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minInput the minimum input value
     * @param maxInput the maximum input value
     * @return this
     */
    public PIDFController enableContinuousInput(double minInput, double maxInput) {
        continuousInput = true;
        minContinuousInput = minInput;
        maxContinuousInput = maxInput;
        return this;
    }

    /**
     * Disables continuous input.
     *
     * @return this
     */
    public PIDFController disableContinuousInput() {
        continuousInput = false;
        return this;
    }

    /**
     * Whether the input is continuous.
     */
    public boolean isContinuousInputEnabled() {
        return continuousInput;
    }

    /**
     * Get the derivative smoothing coefficient.
     *
     * @return the gain for the low pass filter
     */
    public double getDerivativeSmoothingGain() {
        return derivativeFilter.gain;
    }

    /**
     * Set the derivative smoothing coefficient.
     *
     * @param lowPassGain the gain for the low pass filter, {@code 0 < lowPassGain < 1},
     *                    defaults to 0.01 for effectively no smoothing
     * @return this
     */
    public PIDFController setDerivativeSmoothingGain(double lowPassGain) {
        derivativeFilter = new Filter.LowPass(lowPassGain);
        return this;
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     * @return this
     */
    @NonNull
    public PIDFController setSetpoint(double sp) {
        if (clearIOnNewSetpoint && sp != setPoint) {
            errorI = 0;
        }
        setPoint = sp;
        errorP = setPoint - currentPv;
        errorD = (errorP - prevErrorP) / period;
        return this;
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return hasMeasured && Math.abs(errorP) < errorToleranceP
                && Math.abs(errorD) < errorToleranceD;
    }

    /**
     * @return the PIDF coefficients
     */
    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * <b>Note!</b> These coefficients will always be ordered in the following way:
     * kP, kI, kD, kF. Omission of values will drop up to the last value kP for a PController, however, do note
     * in a PD controller the second term will be kI, not kD. PID/PD/P controllers are simply PIDF controllers
     * with omitted values.
     * <p>
     * When using a {@link CompositeController}, do note you will have to supply
     * <b>all four</b> coefficients to preserve ordering with the new composed controller.
     *
     * @inheritDoc
     */
    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length < 1) {
            throw new IllegalArgumentException("expected >1 coefficients, got " + coeffs.length);
        }
        double i = coeffs.length > 1 ? coeffs[1] : 0;
        double d = coeffs.length > 2 ? coeffs[2] : 0;
        double f = coeffs.length > 3 ? coeffs[3] : 0;
        setPIDF(coeffs[0], i, d, f);
    }

    /**
     * @return the tolerances of the controller in order of positional and derivative tolerances
     */
    @NonNull
    public double[] getTolerance() {
        return new double[]{errorToleranceP, errorToleranceD};
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetpoint()}.
     *
     * @param tolerances The positional and derivative tolerances.
     * @return this
     */
    @NonNull
    public PIDFController setTolerance(@NonNull double[] tolerances) {
        setTolerance(tolerances[0], tolerances[1]);
        return this;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetpoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @return this
     */
    @NonNull
    public PIDFController setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
        return this;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetpoint()}.
     *
     * @param positionTolerance   Position error which is tolerable.
     * @param derivativeTolerance Derivative error which is tolerable.
     * @return this
     */
    @NonNull
    public PIDFController setTolerance(double positionTolerance, double derivativeTolerance) {
        errorToleranceP = positionTolerance;
        errorToleranceD = derivativeTolerance;
        return this;
    }

    /**
     * @return the error {@code e(t)} measured by the controller
     */
    public double getError() {
        return errorP;
    }

    /**
     * @return the derivative of the error {@code e'(t)} measured by the controller
     */
    public double getErrorDerivative() {
        return errorD;
    }

    /**
     * @return the integral of the error {@code int(0,t)[e(t')dt']} measured by the controller
     */
    public double getTotalError() {
        return errorI;
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * {@link #calculate(double)}.
     */
    public double calculate() {
        return calculate(currentPv);
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output using the given measured value via
     * {@link #calculate(double)}.
     */
    @Override
    public double calculate(double pv, double sp) {
        // Set the setpoint to the provided value
        setSetpoint(sp);
        return calculate(pv);
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double calculate(double pv) {
        hasMeasured = true;
        prevErrorP = errorP;

        double currentTimeStamp = System.nanoTime() / 1.0E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (currentPv == pv) {
            errorP = setPoint - currentPv;
        } else {
            errorP = setPoint - pv;
            currentPv = pv;
        }
        if (continuousInput) {
            double bound = (maxContinuousInput - minContinuousInput) / 2.0;
            errorP = Mathf.wrap(errorP, -bound, bound);
        }

        if (Math.abs(period) > 1.0E-6) {
            errorD = derivativeFilter.apply(errorP - prevErrorP) / period;
        } else {
            errorD = 0;
        }

        /*
         * If total error is the integral from 0 to t of e(t')dt', and
         * e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        if (Math.abs(errorP) <= iZone) {
            errorI += period * (setPoint - currentPv);
            errorI = errorI < minIClamp ? minIClamp : Math.min(maxIClamp, errorI);
        } else {
            errorI = 0;
        }

        // Returns u(t)
        double ut = kP * errorP + kI * errorI + kD * errorD + kF * setPoint;
        return Mathf.clamp(ut, lowerClamp, upperClamp);
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     * @return this
     */
    @NonNull
    public PIDFController setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        return this;
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param coefficients the coefficients to set
     * @return this
     */
    @NonNull
    public PIDFController setPIDF(@NonNull PIDFCoefficients coefficients) {
        kP = coefficients.p;
        kI = coefficients.i;
        kD = coefficients.d;
        kF = coefficients.f;
        return this;
    }

    /**
     * Update the supplied PID coefficients with the current controller values.
     *
     * @param coefficients the coefficients to update
     * @return this
     */
    @NonNull
    public PIDFController updatePIDF(@NonNull PIDFCoefficients coefficients) {
        coefficients.p = kP;
        coefficients.i = kI;
        coefficients.d = kD;
        coefficients.f = kF;
        return this;
    }

    /**
     * Resets the PIDF controller.
     */
    @Override
    public void reset() {
        hasMeasured = false;
        errorP = 0;
        errorI = 0;
        errorD = 0;
        prevErrorP = 0;
        lastTimeStamp = 0;
    }

    /**
     * Clear the integral term.
     *
     * @return this
     */
    @NonNull
    public PIDFController clearTotalError() {
        errorI = 0;
        return this;
    }

    public double getP() {
        return kP;
    }

    /**
     * Set the Proportional coefficient on this controller
     *
     * @param kp proportional
     * @return this
     */
    @NonNull
    public PIDFController setP(double kp) {
        kP = kp;
        return this;
    }

    public double getI() {
        return kI;
    }

    /**
     * Set the Integral coefficient on this controller
     *
     * @param ki integral
     * @return this
     */
    @NonNull
    public PIDFController setI(double ki) {
        kI = ki;
        return this;
    }

    public double getD() {
        return kD;
    }

    /**
     * Set the Derivative coefficient on this controller
     *
     * @param kd derivative, will be filtered with a low pass filter (see {@link #setDerivativeSmoothingGain(double)})
     * @return this
     */
    @NonNull
    public PIDFController setD(double kd) {
        kD = kd;
        return this;
    }

    public double getF() {
        return kF;
    }

    /**
     * Set the setpoint Feedforward coefficient on this controller
     *
     * @param kf feedforward, will be multiplied by the setpoint
     * @return this
     */
    @NonNull
    public PIDFController setF(double kf) {
        kF = kf;
        return this;
    }

    /**
     * Get the period of the controller.
     *
     * @return the time period of the controller, in seconds
     */
    public double getPeriod() {
        return period;
    }

    @NonNull
    @Override
    public Optional<PIDFController> pidf() {
        return Optional.of(this);
    }
}
