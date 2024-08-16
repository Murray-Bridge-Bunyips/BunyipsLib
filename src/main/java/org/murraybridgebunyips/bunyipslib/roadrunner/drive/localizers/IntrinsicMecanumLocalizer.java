package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.Cartesian;
import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * An estimation-based Mecanum localizer that uses no encoders and only the IMU for heading. This is the bare configuration
 * for a Mecanum drivetrain, and should only be used in an absolute dire situation where all your encoders don't work.
 *
 * @author Lucas Bubner, 2024
 */
public class IntrinsicMecanumLocalizer implements Localizer {
    private final Coefficients coefficients;
    private final MecanumDrive drive;
    private final ElapsedTime timer = new ElapsedTime();
    private final double imuOffset;

    private final double[] lastMotorPos = new double[4];
    private final double[] runningMotorPos = new double[4];
    private double lastHeading = 0;

    private Supplier<Pose2d> input;
    private Supplier<double[]> powers;

    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = new Pose2d();

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
        powers = drive::getMotorPowers;
        if (Storage.memory().lastKnownPosition != null)
            poseEstimate = Storage.memory().lastKnownPosition;
        imuOffset = drive.getExternalHeading();
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
        input = driveInput;
        if (Storage.memory().lastKnownPosition != null)
            poseEstimate = Storage.memory().lastKnownPosition;
        imuOffset = drive.getExternalHeading();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
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

        double heading = Mathf.angleModulus(Radians.of(drive.getExternalHeading() - imuOffset)).in(Radians);
        double headingDelta = Mathf.angleModulus(Radians.of(heading - lastHeading)).in(Radians);

        // Using RoadRunner's MecanumKinematics to calculate the robot pose delta and to apply it to the current pose
        Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas,
                drive.getConstants().TRACK_WIDTH,
                drive.getConstants().TRACK_WIDTH,
                1
        );
        poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), headingDelta));
        poseVelocity = new Pose2d(robotPoseDelta.vec(), drive.getExternalHeadingVelocity());

        // Update for deltas
        System.arraycopy(runningMotorPos, 0, lastMotorPos, 0, 4);
        lastHeading = heading;
        timer.reset();
    }

    private double[] getWheelPowers(Pose2d inputCartesian) {
        double[] inputs = {inputCartesian.getX(), inputCartesian.getY(), inputCartesian.getHeading()};
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
