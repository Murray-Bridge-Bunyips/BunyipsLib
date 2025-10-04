package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;

/**
 * Velocity feedforward gain. Used to counter electromotive force and drag.
 * <p>
 * <b>Note:</b> This feedforward is the exact same as the {@code kF} gain on the {@link PIDFController}.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public class kV implements SystemController {
    private double kV;

    /**
     * Create a new kV gain.
     *
     * @param kV velocity gain
     */
    public kV(double kV) {
        this.kV = kV;
    }

    @Override
    public double calculate(double ignored, double setpoint) {
        return kV * setpoint;
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kV};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 1)
            throw new IllegalArgumentException("expected 1 coefficient, got " + coeffs.length);
        kV = coeffs[0];
    }
}
