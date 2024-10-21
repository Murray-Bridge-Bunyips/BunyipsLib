package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * A PIDF controller with no feedforward.
 * <a href="https://github.com/FTCLib/FTCLib/blob/cedc52cee1bb549324c1ca5059c5feec3c054902/core/src/main/java/com/arcrobotics/ftclib/controller/PIDController.java">Source</a>
 *
 * @since 1.0.0-pre
 */
public class PIDController extends PIDFController {
    /**
     * Default constructor with just the coefficients
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     */
    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd, 0);
    }

    /**
     * The extended constructor.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param sp The setpoint for the controller.
     * @param pv The process variable for the controller.
     */
    public PIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, 0, sp, pv);
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @return this
     */
    @NonNull
    public PIDController setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
        return this;
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param coefficients the coefficients to set
     * @return this
     */
    @NonNull
    public PIDController setPID(@NonNull PIDCoefficients coefficients) {
        setPIDF(coefficients.p, coefficients.i, coefficients.d, 0);
        return this;
    }

    /**
     * Update the supplied PID coefficients with the current controller values.
     *
     * @param coefficients the coefficients to update
     * @return this
     */
    @NonNull
    public PIDController updatePID(@NonNull PIDCoefficients coefficients) {
        coefficients.p = getP();
        coefficients.i = getI();
        coefficients.d = getD();
        return this;
    }
}
