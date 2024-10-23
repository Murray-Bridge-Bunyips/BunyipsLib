package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * An estimation-based Mecanum localizer that uses no encoders and only the IMU for heading. This is the bare configuration
 * for a Mecanum drivetrain, and should only be used in an absolute dire situation where all your encoders don't work.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class IntrinsicMecanumLocalizer implements Localizer {
    private final Params params;
    private final ElapsedTime timer = new ElapsedTime();
    private final MecanumKinematics kinematics;
    private final IMULocalizer imu;
    private final double[] lastMotorPos = new double[4];
    private final double[] runningMotorPos = new double[4];
    private final Supplier<double[]> powers;

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param params the coefficients used in calculating the intrinsic Mecanum pose
     * @param imu    the IMU to use for heading information
     * @param motorPowers a four-wide double array supplier that supplies the current motor powers, in the form
     *                    {frontLeft, backLeft, backRight, frontRight}
     */
    public IntrinsicMecanumLocalizer(@NonNull Params params, @NonNull IMU imu, @NonNull Supplier<double[]> motorPowers) {
        this.params = params;
        this.imu = new IMULocalizer(imu);
        kinematics = new MecanumKinematics(1);
        powers = motorPowers;
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        double[] wheelInputs = powers.get();
        for (int i = 0; i < 4; i++) {
            wheelInputs[i] = Math.abs(wheelInputs[i]) >= params.BREAKAWAY_SPEEDS[i % 2] ? wheelInputs[i] : 0;
        }

        // Use a delta time strategy to calculate a running total
        double deltaTime = timer.seconds();
        for (int i = 0; i < 4; i++) {
            runningMotorPos[i] += wheelInputs[i] * deltaTime * params.MULTIPLIER;
        }

        List<Double> wheelDeltas = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            wheelDeltas.add(runningMotorPos[i] - lastMotorPos[i]);
        }

        Twist2dDual<Time> robotPoseDelta = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                DualNum.constant(wheelDeltas.get(0), 2),
                DualNum.constant(wheelDeltas.get(1), 2),
                DualNum.constant(wheelDeltas.get(2), 2),
                DualNum.constant(wheelDeltas.get(3), 2)
        ));

        // Update for deltas
        System.arraycopy(runningMotorPos, 0, lastMotorPos, 0, 4);
        timer.reset();

        return new Twist2dDual<>(
                robotPoseDelta.line,
                imu.update().angle
        );
    }

    /**
     * Parameters used in calculating the intrinsic Mecanum pose.
     */
    public static class Params {
        /**
         * The general multiplier for the Mecanum vector estimation.
         */
        public double MULTIPLIER = 1;

        /**
         * The breakaway speeds for the forward and strafe axes.
         */
        @NonNull
        public double[] BREAKAWAY_SPEEDS = {0.1, 0.1};

        /**
         * Utility builder for intrinsic Mecanum pose estimation coefficients.
         */
        public static class Builder {
            private final Params params = new Params();

            /**
             * Set the general wheel multiplier for the Mecanum vector estimation.
             *
             * @param multiplier the general multiplier for wheel positions. Will need empirical tuning in the same
             *                   way you would tune the multiplier for a localizer, by measuring the actual distance
             *                   travelled over a distance divided by the reported distance.
             * @return this builder
             */
            @NonNull
            public Builder setMultiplier(double multiplier) {
                params.MULTIPLIER = multiplier;
                return this;
            }

            /**
             * Set the breakaway powers for the forward and strafe axes.
             * The breakaway powers are the minimum power required to move the robot in a given direction.
             *
             * @param forward the minimum vector power required to move the robot forward
             * @param strafe  the minimum vector power required to move the robot left/right
             * @return this builder
             */
            @NonNull
            public Builder setBreakawaySpeeds(double forward, double strafe) {
                params.BREAKAWAY_SPEEDS = new double[]{forward, strafe};
                return this;
            }

            /**
             * Build the coefficients.
             *
             * @return the coefficients
             */
            @NonNull
            public Params build() {
                return params;
            }
        }
    }
}
