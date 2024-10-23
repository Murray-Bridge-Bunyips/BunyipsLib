package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.MecanumDrive;

/**
 * An estimation-based Mecanum localizer that uses no encoders and only the IMU for heading. This is the bare configuration
 * for a Mecanum drivetrain, and should only be used in an absolute dire situation where all your encoders don't work.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class IntrinsicMecanumLocalizer implements Localizer {
    private final Params params;
    private final MecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();
    private final MecanumKinematics kinematics;
    private final double imuOffset;

    private final double[] lastMotorPos = new double[4];
    private final double[] runningMotorPos = new double[4];
    private double lastHeading = 0;

    private Supplier<PoseVelocity2d> input;
    private Supplier<double[]> powers;

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param params the coefficients used in calculating the intrinsic Mecanum pose
     * @param drive        the drive instance (assumed to be Mecanum/conforms to Mecanum equations). Power info will be
     *                     extracted from the drive.
     */
    public IntrinsicMecanumLocalizer(@NonNull Params params, @NonNull MecanumDrive drive) {
        this.params = params;
        this.drive = drive;
        DriveModel model = drive.getConstants().getDriveModel();
        kinematics = new MecanumKinematics(model.inPerTick * model.trackWidthTicks,
                model.inPerTick / model.lateralInPerTick);
        powers = drive::getMotorPowers;
        imuOffset = drive.getPose().heading.toDouble();
    }

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param params the coefficients used in calculating the intrinsic Mecanum pose
     * @param drive        the drive instance (assumed to be Mecanum/conforms to Mecanum equations)
     * @param driveInput   the robot pose input (x forward, y left, heading anticlockwise) of the drive
     */
    public IntrinsicMecanumLocalizer(@NonNull Params params, @NonNull MecanumDrive drive, @NonNull Supplier<PoseVelocity2d> driveInput) {
        this.params = params;
        this.drive = drive;
        DriveModel model = drive.getConstants().getDriveModel();
        kinematics = new MecanumKinematics(model.inPerTick * model.trackWidthTicks,
                model.inPerTick / model.lateralInPerTick);
        input = driveInput;
        imuOffset = drive.getPose().heading.toDouble();
    }

    @NonNull
    @Override
    public Twist2dDual<Time> update() {
        double[] wheelInputs = input != null
                ? forwardKinematics(input.get())
                : getClampedPowers(powers.get());

        // Use a delta time strategy to calculate a running total
        double deltaTime = timer.seconds();
        for (int i = 0; i < 4; i++) {
            runningMotorPos[i] += wheelInputs[i] * deltaTime * params.MULTIPLIER;
        }

        List<Double> wheelDeltas = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            wheelDeltas.add(runningMotorPos[i] - lastMotorPos[i]);
        }

        double heading = Mathf.angleModulus(Radians.of(drive.getPose().heading.toDouble() - imuOffset)).in(Radians);
        double headingDelta = Mathf.angleModulus(Radians.of(heading - lastHeading)).in(Radians);

        Twist2dDual<Time> robotPoseDelta = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                DualNum.constant(wheelDeltas.get(0), 2),
                DualNum.constant(wheelDeltas.get(1), 2),
                DualNum.constant(wheelDeltas.get(2), 2),
                DualNum.constant(wheelDeltas.get(3), 2)
        ));

        // Update for deltas
        System.arraycopy(runningMotorPos, 0, lastMotorPos, 0, 4);
        lastHeading = heading;
        timer.reset();

        return new Twist2dDual<>(
                robotPoseDelta.line,
                DualNum.cons(headingDelta, robotPoseDelta.angle.drop(1))
        );
    }

    private double[] forwardKinematics(PoseVelocity2d input) {
        double[] cartesianInputs = {input.linearVel.y, -input.linearVel.x, -input.angVel};
        // Should reject anything below the breakaway speeds
        double cX = Math.abs(cartesianInputs[0]) >= params.BREAKAWAY_SPEEDS[1] ? cartesianInputs[0] : 0;
        double cY = Math.abs(cartesianInputs[1]) >= params.BREAKAWAY_SPEEDS[0] ? cartesianInputs[1] : 0;
        double cR = Math.abs(cartesianInputs[2]) >= params.BREAKAWAY_SPEEDS[2] ? cartesianInputs[2] : 0;

        // Standard Mecanum drive equations
        double denom = Math.max(Math.abs(cY) + Math.abs(cX) + Math.abs(cR), 1);
        return new double[]{
                (cY + cX + cR) / denom, // Front left
                (cY - cX + cR) / denom, // Back left
                (cY + cX - cR) / denom, // Back right
                (cY - cX - cR) / denom, // Front right
        };
    }

    private double[] getClampedPowers(double[] wheelPowers) {
        // We already have the powers, so all we need to do is clamp
        // with an alternation of forward and strafe breakaway speeds
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = Math.abs(wheelPowers[i]) >= params.BREAKAWAY_SPEEDS[i % 2] ? wheelPowers[i] : 0;
        }
        return wheelPowers;
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
