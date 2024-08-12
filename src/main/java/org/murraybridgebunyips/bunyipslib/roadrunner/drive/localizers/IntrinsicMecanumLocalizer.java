package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.InchesPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.murraybridgebunyips.bunyipslib.Cartesian;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.function.Supplier;

/**
 * An estimation-based Mecanum localizer that uses no encoders and only the IMU for heading. This is the bare configuration
 * for a Mecanum drivetrain, and should only be used in an absolute dire situation where all your encoders don't work.
 *
 * @author Lucas Bubner, 2024
 */
/* TODO: Work in progress */ class IntrinsicMecanumLocalizer implements Localizer {
    private final RoadRunnerDrive drive;
    private final Supplier<Pose2d> input;

    private Pose2d poseEstimate = new Pose2d();

    /**
     * Create a new IntrinsicMecanumLocalizer.
     *
     * @param driveInput the robot pose input (x forward, y left, heading anticlockwise) of the drive
     * @param drive the drive instance (assumed to be Mecanum/conforms to Mecanum equations)
     */
    public IntrinsicMecanumLocalizer(Supplier<Pose2d> driveInput, RoadRunnerDrive drive) {
        input = driveInput;
        this.drive = drive;
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
        return null;
    }

    @Override
    public void update() {
        Pose2d inputCartesian = Cartesian.fromPose(input.get());
        double denom = Math.max(Math.abs(inputCartesian.getY()) + Math.abs(inputCartesian.getX()) + Math.abs(inputCartesian.getHeading()), 1);
        double fl = (inputCartesian.getY() + inputCartesian.getX() + inputCartesian.getHeading()) / denom;
        double bl = (inputCartesian.getY() - inputCartesian.getX() + inputCartesian.getHeading()) / denom;
        double fr = (inputCartesian.getY() - inputCartesian.getX() - inputCartesian.getHeading()) / denom;
        double br = (inputCartesian.getY() + inputCartesian.getX() - inputCartesian.getHeading()) / denom;
    }

    /**
     * Coefficients used in calculating the intrinsic Mecanum pose.
     */
    public static class Coefficients {
        public double FULL_POWER_VEL = 1; // Will likely be reworked, these are here for the meantime

        public double BREAKAWAY_POWER = 0.3;

        /**
         * Utility builder for intrinsic Mecanum pose estimation coefficients.
         */
        public static class Builder {
            private final IntrinsicMecanumLocalizer.Coefficients coeffs = new IntrinsicMecanumLocalizer.Coefficients();

            public Builder setFullPowerVel(Measure<Velocity<Distance>> maxVelocity) {
                coeffs.FULL_POWER_VEL = maxVelocity.in(InchesPerSecond);
                return this;
            }

            public Builder setBreakawayPower(double power) {
                coeffs.BREAKAWAY_POWER = Mathf.clamp(power, -1, 1);
                return this;
            }

            public IntrinsicMecanumLocalizer.Coefficients build() {
                return coeffs;
            }
        }
    }
}
