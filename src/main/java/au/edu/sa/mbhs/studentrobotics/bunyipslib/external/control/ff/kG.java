package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;

/**
 * Constant feedforward gain. Used to overcome constant forces such as gravity.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public class kG implements SystemController {
    private double kG;

    /**
     * Create a new kG gain.
     *
     * @param kG gravity gain
     */
    public kG(double kG) {
        this.kG = kG;
    }

    @Override
    public double calculate(double ignored, double setpoint) {
        return kG;
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kG};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 1)
            throw new IllegalArgumentException("expected 1 coefficient, got " + coeffs.length);
        kG = coeffs[0];
    }
}
