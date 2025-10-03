package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;

/**
 * Static feedforward gain. Used to overcome friction.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public class kS implements SystemController {
    private double kS;

    /**
     * Create a new kS gain.
     *
     * @param kS static gain
     */
    public kS(double kS) {
        this.kS = kS;
    }

    @Override
    public double calculate(double ignored, double setpoint) {
        return kS * Math.signum(setpoint);
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kS};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 1)
            throw new IllegalArgumentException("expected 1 coefficient, got " + coeffs.length);
        kS = coeffs[0];
    }
}
