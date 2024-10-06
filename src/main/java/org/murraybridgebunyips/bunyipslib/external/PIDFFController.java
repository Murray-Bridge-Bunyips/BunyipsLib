package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.Encoder;
import org.murraybridgebunyips.bunyipslib.Motor;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;

import java.util.Arrays;

/**
 * A composite controller to use for PID+FF controls (with kV/kA calculated components). This controller is compatible
 * with any error-correction need, including RUN_TO_POSIITON and RUN_USING_ENCODER modes in {@link Motor}.
 * <p>
 * To use an ArmFeedforward, consider using the {@link ArmController}, which will properly integrate the cosine of
 * the arm position as part of your feedforward during a RUN_TO_POSITION.
 *
 * @author Lucas Bubner, 2024
 * @see ArmController
 * @since 4.0.0
 */
public class PIDFFController implements PIDF {
    private final PIDF pid;
    private final Encoder encoder;
    private final SystemController ff;

    /**
     * Construct a new PIDFFController.
     *
     * @param pid     the PID controller to use
     * @param ff      the kV/kA feedforward to use
     * @param encoder the encoder to retrieve velocity/acceleration information from
     */
    public PIDFFController(PIDF pid, SystemController ff, Encoder encoder) {
        this.pid = pid;
        this.ff = ff;
        this.encoder = encoder;
    }

    @Override
    public double[] getCoefficients() {
        double[] ffCoeffs = ff.getCoefficients();
        double[] pidCoeffs = pid.getCoefficients();

        double[] coeffs = new double[ffCoeffs.length + pidCoeffs.length];
        System.arraycopy(pidCoeffs, 0, coeffs, 0, pidCoeffs.length);
        System.arraycopy(ffCoeffs, 0, coeffs, pidCoeffs.length, ffCoeffs.length);

        return coeffs;
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length < 4) {
            throw new IllegalArgumentException("expected >=4 arguments, got " + coeffs.length);
        }
        pid.getPIDFController().setPIDF(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
        ff.setCoefficients(Arrays.stream(coeffs).skip(4).toArray());
    }

    @Override
    public double calculate(double current, double target) {
        return pid.calculate(current, target) + ff.calculate(encoder.getVelocity(), encoder.getAcceleration());
    }

    /**
     * Alternative calculate method where the feedforward component will be assumed to be a velocity, allowing
     * your feedforward to react faster. This is internally called in {@link Motor} via a type check.
     *
     * @param currentVelo current velocity
     * @param targetVelo  target velocity, will be used instead of the encoder velocity for feedforward
     * @return calculated controller output
     */
    public double calculateVelo(double currentVelo, double targetVelo) {
        return pid.calculate(currentVelo, targetVelo) + ff.calculate(targetVelo, encoder.getAcceleration());
    }

    @Override
    public void reset() {
        pid.reset();
        ff.reset();
    }

    @Override
    public PIDFController getPIDFController() {
        return pid.getPIDFController();
    }
}
