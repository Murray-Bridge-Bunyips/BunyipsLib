package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Optional;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;

/**
 * This is a PID controller (https://en.wikipedia.org/wiki/PID_controller)
 * for your robot. Internally, it performs all the calculations for you.
 * You need to tune your values to the appropriate amounts in order
 * to properly utilize these calculations.
 * <p>
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 * <a href="https://github.com/FTCLib/FTCLib/blob/cedc52cee1bb549324c1ca5059c5feec3c054902/core/src/main/java/com/arcrobotics/ftclib/controller/PIDFController.java">Source</a>
 *
 * @since 1.0.0-pre
 */
public class PIDFController implements SystemController {
    private double kP, kI, kD, kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    private double lowerLim = -Double.MAX_VALUE;
    private double upperLim = Double.MAX_VALUE;

    /**
     * The base constructor for the PIDF controller
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
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
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
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    /**
     * Resets the PIDF controller.
     */
    @Override
    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     * @return this
     */
    @NonNull
    public PIDFController setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
        return this;
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
        lowerLim = lower;
        upperLim = upper;
        return this;
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     * @return this
     */
    @NonNull
    public PIDFController setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = setPoint - measuredValue;
        errorVal_v = (errorVal_p - prevErrorVal) / period;
        return this;
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
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
     * @return the positional error e(t)
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * @return the tolerances of the controller
     */
    @NonNull
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param tolerances The positional and velocity tolerances.
     * @return this
     */
    @NonNull
    public PIDFController setTolerance(@NonNull double[] tolerances) {
        setTolerance(tolerances[0], tolerances[1]);
        return this;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
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
     * @return the velocity error e'(t)
     */
    public double getVelocityError() {
        return errorVal_v;
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * {@link #calculate(double)}.
     */
    public double calculate() {
        return calculate(measuredValue);
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
        setSetPoint(sp);
        return calculate(pv);
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = System.nanoTime() / 1.0E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (measuredValue == pv) {
            errorVal_p = setPoint - measuredValue;
        } else {
            errorVal_p = setPoint - pv;
            measuredValue = pv;
        }

        if (Math.abs(period) > 1.0E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        /*
         * If total error is the integral from 0 to t of e(t')dt', and
         * e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // Returns u(t)
        double ut = kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;
        return Mathf.clamp(ut, lowerLim, upperLim);
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
     * Set the bounds for the integral term.
     *
     * @param integralMin The minimum value for the integral term.
     * @param integralMax The maximum value for the integral term.
     * @return this
     */
    @NonNull
    public PIDFController setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
        return this;
    }

    /**
     * Clear the integral term.
     *
     * @return this
     */
    @NonNull
    public PIDFController clearTotalError() {
        totalError = 0;
        return this;
    }

    public double getP() {
        return kP;
    }

    /**
     * Set the P coefficient on this controller
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
     * Set the I coefficient on this controller
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
     * Set the D coefficient on this controller
     *
     * @param kd derivative
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
     * Set the F coefficient on this controller
     *
     * @param kf feedforward
     * @return this
     */
    @NonNull
    public PIDFController setF(double kf) {
        kF = kf;
        return this;
    }

    public double getPeriod() {
        return period;
    }

    @NonNull
    @Override
    public Optional<PIDFController> pidf() {
        return Optional.of(this);
    }
}
