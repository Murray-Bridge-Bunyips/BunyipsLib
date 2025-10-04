package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.EncoderTicks;

/**
 * Cosine feedforward gain. Used to overcome non-linear dynamics in rotation.
 * <p>
 * Exposes two constructors for applying cosine on the setpoint or on the current angular position of the system.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public class kCos implements SystemController {
    private double kCos;
    private UnaryFunction setpointToRadians;
    private Supplier<Measure<Angle>> angle;

    /**
     * Create a new kCos on the setpoint.
     *
     * @param kCos              the kCos gain
     * @param setpointToRadians conversion function to convert the supplied setpoint (usually in motor ticks) to radians, see {@link EncoderTicks}
     */
    public kCos(double kCos, UnaryFunction setpointToRadians) {
        this.kCos = kCos;
        this.setpointToRadians = setpointToRadians;
    }

    /**
     * Create a new kCos on the dynamic angular position of the system.
     *
     * @param kCos  the kCos gain
     * @param angle current angular state of the system, see {@link EncoderTicks}
     */
    public kCos(double kCos, Supplier<Measure<Angle>> angle) {
        this.kCos = kCos;
        this.angle = angle;
    }

    @Override
    public double calculate(double ignored, double setpoint) {
        return kCos * Math.cos(setpointToRadians != null ? setpointToRadians.apply(setpoint) : angle.get().in(Radians));
    }

    @NonNull
    @Override
    public double[] getCoefficients() {
        return new double[]{kCos};
    }

    @Override
    public void setCoefficients(@NonNull double[] coeffs) {
        if (coeffs.length != 1)
            throw new IllegalArgumentException("expected 1 coefficient, got " + coeffs.length);
        kCos = coeffs[0];
    }
}
