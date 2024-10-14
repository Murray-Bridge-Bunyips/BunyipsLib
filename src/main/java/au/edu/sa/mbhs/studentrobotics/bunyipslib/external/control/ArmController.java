package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.EncoderTicks;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.ff.ArmFeedforward;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;

import java.util.function.Supplier;

/**
 * A composite PID and Feedforward controller that represents an arm suspended at an angle from gravity. This class
 * is similar to {@link PIDFFController}, but uses suppliers for angles to be used in calculating feedforward, allowing
 * proper calculation of the feedforward cosine coefficient.
 *
 * @author Lucas Bubner, 2024
 * @see EncoderTicks
 * @see PIDFFController
 * @since 4.0.0
 */
public class ArmController implements PIDF {
    private final PIDF pid;
    private final ArmFeedforward ff;
    private final Supplier<Measure<Angle>> setPointAngleProvider;
    private final Supplier<Measure<Velocity<Angle>>> velocityAngleProvider;
    private final Supplier<Measure<Velocity<Velocity<Angle>>>> accelerationAngleProvider;

    /**
     * Construct a new ArmController.
     *
     * @param pid                       the PID controller to use
     * @param ff                        the ArmFeedforward controller to use
     * @param setPointAngleProvider     the setpoint angle provider
     * @param velocityAngleProvider     the angular velocity provider
     * @param accelerationAngleProvider the angular acceleration provider
     */
    public ArmController(PIDF pid, ArmFeedforward ff, Supplier<Measure<Angle>> setPointAngleProvider, Supplier<Measure<Velocity<Angle>>> velocityAngleProvider, Supplier<Measure<Velocity<Velocity<Angle>>>> accelerationAngleProvider) {
        this.pid = pid;
        this.ff = ff;
        this.setPointAngleProvider = setPointAngleProvider;
        this.velocityAngleProvider = velocityAngleProvider;
        this.accelerationAngleProvider = accelerationAngleProvider;
    }

    @Override
    public double[] getCoefficients() {
        PIDFController c = pid.getPIDFController();
        return new double[]{c.getP(), c.getI(), c.getD(), c.getF(), ff.getS(), ff.getCos(), ff.getV(), ff.getA()};
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length != 8) {
            throw new IllegalArgumentException("expected 8 coefficients, got " + coeffs.length);
        }
        pid.getPIDFController().setPIDF(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
        ff.setCoefficients(coeffs[4], coeffs[5], coeffs[6], coeffs[7]);
    }

    @Override
    public double calculate(double current, double target) {
        return pid.calculate(current, target) + ff.calculate(setPointAngleProvider.get(), velocityAngleProvider.get(), accelerationAngleProvider.get());
    }

    @Override
    public void reset() {
        pid.reset();
    }

    @Override
    public PIDFController getPIDFController() {
        return pid.getPIDFController();
    }
}
