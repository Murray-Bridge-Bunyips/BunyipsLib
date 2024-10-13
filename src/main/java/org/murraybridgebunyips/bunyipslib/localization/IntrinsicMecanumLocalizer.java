package org.murraybridgebunyips.bunyipslib.localization;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.util.Cartesian;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.subsystems.drive.MecanumDrive;

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
    private final Coefficients coefficients;
    private final MecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();
    private final MecanumKinematics kinematics;
    private final double imuOffset;

    private final double[] lastMotorPos = new double[4];
    private final double[] runningMotorPos = new double[4];
    private double lastHeading = 0;

    private Supplier<Pose2d> input;
    private Supplier<double[]> powers;

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param coefficients the coefficients used in calculating the intrinsic Mecanum pose
     * @param drive        the drive instance (assumed to be Mecanum/conforms to Mecanum equations). Power info will be
     *                     extracted from the drive.
     */
    public IntrinsicMecanumLocalizer(Coefficients coefficients, MecanumDrive drive) {
        this.coefficients = coefficients;
        this.drive = drive;
        kinematics = new MecanumKinematics(drive.getDriveModel().inPerTick * drive.getDriveModel().trackWidthTicks,
                drive.getDriveModel().inPerTick / drive.getDriveModel().lateralInPerTick);
        powers = drive::getMotorPowers;
        imuOffset = drive.getPoseEstimate().heading.toDouble();
    }

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param coefficients the coefficients used in calculating the intrinsic Mecanum pose
     * @param drive        the drive instance (assumed to be Mecanum/conforms to Mecanum equations)
     * @param driveInput   the robot pose input (x forward, y left, heading anticlockwise) of the drive
     */
    public IntrinsicMecanumLocalizer(Coefficients coefficients, MecanumDrive drive, Supplier<Pose2d> driveInput) {
        this.coefficients = coefficients;
        this.drive = drive;
        kinematics = new MecanumKinematics(drive.getDriveModel().inPerTick * drive.getDriveModel().trackWidthTicks,
                drive.getDriveModel().inPerTick / drive.getDriveModel().lateralInPerTick);
        input = driveInput;
        imuOffset = drive.getPoseEstimate().heading.toDouble();
    }

    @Override
    public Twist2dDual<Time> update() {
        double[] wheelInputs = input != null
                ? getWheelPowers(Cartesian.fromPose(input.get()))
                : getClampedPowers(powers.get());

        // Use a delta time strategy to calculate a running total
        double deltaTime = timer.seconds();
        for (int i = 0; i < 4; i++) {
            runningMotorPos[i] += wheelInputs[i] * deltaTime * coefficients.MULTIPLIER;
        }

        List<Double> wheelDeltas = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            wheelDeltas.add(runningMotorPos[i] - lastMotorPos[i]);
        }

        double heading = Mathf.angleModulus(Radians.of(drive.getPoseEstimate().heading.toDouble() - imuOffset)).in(Radians);
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

    private double[] getWheelPowers(Pose2d inputCartesian) {
        double[] inputs = {inputCartesian.position.x, inputCartesian.position.y, inputCartesian.heading.toDouble()};
        // Should reject anything below the breakaway speeds
        double cX = Math.abs(inputs[0]) >= coefficients.BREAKAWAY_SPEEDS[1] ? inputs[0] : 0;
        double cY = Math.abs(inputs[1]) >= coefficients.BREAKAWAY_SPEEDS[0] ? inputs[1] : 0;
        double cR = Math.abs(inputs[2]) >= coefficients.BREAKAWAY_SPEEDS[2] ? inputs[2] : 0;

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
            wheelPowers[i] = Math.abs(wheelPowers[i]) >= coefficients.BREAKAWAY_SPEEDS[i % 2] ? wheelPowers[i] : 0;
        }
        return wheelPowers;
    }

    /**
     * Coefficients used in calculating the intrinsic Mecanum pose.
     */
    public static class Coefficients {
        /**
         * The general multiplier for the Mecanum vector estimation.
         */
        public double MULTIPLIER = 1;

        /**
         * The breakaway speeds for the forward and strafe axes.
         */
        public double[] BREAKAWAY_SPEEDS = {0.1, 0.1};

        /**
         * Utility builder for intrinsic Mecanum pose estimation coefficients.
         */
        public static class Builder {
            private final Coefficients coeffs = new Coefficients();

            /**
             * Set the general multiplier for the Mecanum vector estimation.
             *
             * @param multiplier the general multiplier for wheel positions. Will need empirical tuning in the same
             *                   way you would tune the multiplier for a deadwheel localizer.
             * @return this builder
             */
            public Builder setMultiplier(double multiplier) {
                coeffs.MULTIPLIER = multiplier;
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
            public Builder setBreakawaySpeeds(double forward, double strafe) {
                coeffs.BREAKAWAY_SPEEDS = new double[]{forward, strafe};
                return this;
            }

            /**
             * Build the coefficients.
             *
             * @return the coefficients
             */
            public Coefficients build() {
                return coeffs;
            }
        }
    }
}
