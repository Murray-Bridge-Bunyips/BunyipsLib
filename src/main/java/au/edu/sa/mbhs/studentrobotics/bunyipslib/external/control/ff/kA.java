package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Diff;

/**
 * Acceleration feedforward gain. Used to induce acceleration based on changes over time to the setpoint.
 * <p>
 * Differs from a conventional {@code kA} gain applied to a target acceleration, rather, attempts to apply a gain
 * to the rate of change of the setpoint.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public class kA implements SystemController {
    private final Diff delta = new Diff();
    private double kA;

    /**
     * Create a new kA gain.
     *
     * @param kA acceleration gain
     */
    public kA(double kA) {
        this.kA = kA;
    }

    @Override
    public double calculate(double ignored, double setpoint) {
        return kA * delta.apply(setpoint);
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kA};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 1)
            throw new IllegalArgumentException("expected 1 coefficient, got " + coeffs.length);
        kA = coeffs[0];
    }

    @Override
    public void reset() {
        delta.reset();
    }
}
