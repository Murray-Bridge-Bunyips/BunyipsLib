package org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Standard 3 tracking wheel localizer implementation.
 *
 * @since 4.0.0
 */
/*
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 */
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private final ElapsedTime imuResetTimer = new ElapsedTime();
    private final ThreeWheelLocalizer.Coefficients coefficients;
    private final Deadwheel leftDeadwheel;
    private final Deadwheel rightDeadwheel;
    private final Deadwheel frontDeadwheel;
    private final List<Integer> lastEncPositions;
    private final List<Integer> lastEncVels;
    private final double xMul;
    private final double yMul;
    @Nullable
    private IMU imu;
    private boolean usingOverflowCompensation;

    /**
     * Create a new ThreeWheelLocalizer with coefficients and encoders. Omits last known encoder value parameters.
     *
     * @param coefficients   The coefficients for the localizer
     * @param leftDeadwheel  The left encoder
     * @param rightDeadwheel The right encoder
     * @param frontDeadwheel The front encoder
     */
    public ThreeWheelLocalizer(ThreeWheelLocalizer.Coefficients coefficients, Deadwheel leftDeadwheel, Deadwheel rightDeadwheel, Deadwheel frontDeadwheel) {
        this(coefficients, leftDeadwheel, rightDeadwheel, frontDeadwheel, new ArrayList<>(), new ArrayList<>());
    }

    /**
     * Create a new ThreeWheelLocalizer with coefficients, encoders, and last encoder positions and velocities.
     *
     * @param coefficients             The coefficients for the localizer
     * @param leftDeadwheel            The left encoder
     * @param rightDeadwheel           The right encoder
     * @param frontDeadwheel           The front encoder
     * @param lastTrackingEncPositions The last encoder positions
     * @param lastTrackingEncVels      The last encoder velocities
     */
    public ThreeWheelLocalizer(ThreeWheelLocalizer.Coefficients coefficients, Deadwheel leftDeadwheel, Deadwheel rightDeadwheel, Deadwheel frontDeadwheel, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, coefficients.LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -coefficients.LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(coefficients.FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.coefficients = coefficients;
        if (this.coefficients.USE_CORRECTED_COUNTS)
            enableOverflowCompensation();

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        assert leftDeadwheel != null && rightDeadwheel != null && frontDeadwheel != null;

        this.leftDeadwheel = leftDeadwheel;
        this.rightDeadwheel = rightDeadwheel;
        this.frontDeadwheel = frontDeadwheel;

        xMul = coefficients.X_MULTIPLIER;
        yMul = coefficients.Y_MULTIPLIER;
    }

    /**
     * Set the IMU that will be used in combination with a IMU relocalization interval.
     *
     * @param imu the imu to use, note that not passing an IMU and activating the relocalization interval
     *            will no-op and propagate a DS warning
     * @return this
     */
    public ThreeWheelLocalizer withRelocalizingIMU(@Nullable IMU imu) {
        this.imu = imu;
        return this;
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public ThreeWheelLocalizer enableOverflowCompensation() {
        usingOverflowCompensation = true;
        return this;
    }

    public ThreeWheelLocalizer.Coefficients getCoefficients() {
        return coefficients;
    }

    /**
     * Convert encoder ticks to inches.
     *
     * @param ticks The encoder ticks
     * @return The inches traveled
     */
    public double encoderTicksToInches(double ticks) {
        return coefficients.WHEEL_RADIUS * 2 * Math.PI * coefficients.GEAR_RATIO * ticks / coefficients.TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        if (coefficients.IMU_RELOCALIZATION_INTERVAL != null) {
            if (imu == null) {
                RobotLog.setGlobalErrorMsg("[ThreeWheelLocalizer] An IMU relocalization interval was set, however, an IMU has not been supplied to the localizer via withRelocalizingIMU(...)\nPlease set the IMU on the localizer to support IMU relocalization.");
            } else if (imuResetTimer.seconds() >= coefficients.IMU_RELOCALIZATION_INTERVAL.in(Seconds)) {
                setPoseEstimate(new Pose2d(getPoseEstimate().vec(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
                imuResetTimer.reset();
            }
        }

        int leftPos = leftDeadwheel.getCurrentPosition();
        int rightPos = rightDeadwheel.getCurrentPosition();
        int frontPos = frontDeadwheel.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * xMul,
                encoderTicksToInches(rightPos) * xMul,
                encoderTicksToInches(frontPos) * yMul
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = usingOverflowCompensation ? (int) leftDeadwheel.getCorrectedVelocity() : (int) leftDeadwheel.getRawVelocity();
        int rightVel = usingOverflowCompensation ? (int) rightDeadwheel.getCorrectedVelocity() : (int) rightDeadwheel.getRawVelocity();
        int frontVel = usingOverflowCompensation ? (int) frontDeadwheel.getCorrectedVelocity() : (int) frontDeadwheel.getRawVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * xMul,
                encoderTicksToInches(rightVel) * xMul,
                encoderTicksToInches(frontVel) * yMul
        );
    }

    /**
     * Constants for RoadRunner standard tracking wheels.
     * Reworked to use a builder for multiple robot configurations.
     *
     * @author Lucas Bubner, 2023
     */
    public static class Coefficients {
        /**
         * Ticks per revolution of the tracking wheel encoders.
         */
        public double TICKS_PER_REV;
        /**
         * Radius of the tracking wheels in inches.
         */
        public double WHEEL_RADIUS = 2;
        /**
         * Gear ratio of the tracking wheels. Calculated as {@code output (wheel) speed / input (encoder) speed}.
         */
        public double GEAR_RATIO = 1;
        /**
         * Inches of lateral distance between the left and right wheels.
         */
        public double LATERAL_DISTANCE = 10;
        /**
         * Forward offset of the lateral wheel to the center of rotation.
         */
        public double FORWARD_OFFSET = 4;
        /**
         * Multiplicative scale of the ticks from the x (forward) axis.
         */
        public double X_MULTIPLIER = 1;
        /**
         * Multiplicative scale of the ticks from the y (strafe) axis.
         */
        public double Y_MULTIPLIER = 1;
        /**
         * Whether to use corrected overflow counts if the TPS exceeds 32767.
         */
        public boolean USE_CORRECTED_COUNTS = false;
        /**
         * The interval between IMU calls to recalculate the heading.
         * This must be combined with a {@link #withRelocalizingIMU(IMU)} call on the ThreeWheelLocalizer if non-null.
         */
        @Nullable
        public Measure<Time> IMU_RELOCALIZATION_INTERVAL = null;

        /**
         * Utility builder for creating new coefficients.
         */
        public static class Builder {
            private final ThreeWheelLocalizer.Coefficients trackingWheelCoefficients;

            /**
             * Start building.
             */
            public Builder() {
                trackingWheelCoefficients = new ThreeWheelLocalizer.Coefficients();
            }

            /**
             * Set the ticks per revolution of the tracking wheel encoder.
             *
             * @param ticksPerRev The ticks per revolution
             * @return The builder
             */
            public Builder setTicksPerRev(double ticksPerRev) {
                trackingWheelCoefficients.TICKS_PER_REV = ticksPerRev;
                return this;
            }

            /**
             * Set the radius of the tracking wheel.
             *
             * @param wheelRadius The radius of the tracking wheel
             * @return The builder
             */
            public Builder setWheelRadius(Measure<Distance> wheelRadius) {
                trackingWheelCoefficients.WHEEL_RADIUS = wheelRadius.in(Inches);
                return this;
            }

            /**
             * Set the gear ratio of the tracking wheel.
             *
             * @param gearRatio The gear ratio of the tracking wheel (output wheel speed / input encoder speed)
             * @return The builder
             */
            public Builder setGearRatio(double gearRatio) {
                trackingWheelCoefficients.GEAR_RATIO = gearRatio;
                return this;
            }

            /**
             * Set the lateral distance between the left and right tracking wheels.
             *
             * @param lateralDistance The lateral distance between the left and right wheels
             * @return The builder
             */
            public Builder setLateralDistance(Measure<Distance> lateralDistance) {
                trackingWheelCoefficients.LATERAL_DISTANCE = lateralDistance.in(Inches);
                return this;
            }

            /**
             * Set the forward offset of the lateral tracking wheel to the center of rotation.
             *
             * @param forwardOffset The forward offset of the lateral tracking wheel
             * @return The builder
             */
            public Builder setForwardOffset(Measure<Distance> forwardOffset) {
                trackingWheelCoefficients.FORWARD_OFFSET = forwardOffset.in(Inches);
                return this;
            }

            /**
             * Set overflow compensation to be used on the localizer.
             *
             * @param correctEncoderCounts whether to use overflow compensation
             * @return The builder
             */
            public Builder setOverflowCompensation(boolean correctEncoderCounts) {
                trackingWheelCoefficients.USE_CORRECTED_COUNTS = correctEncoderCounts;
                return this;
            }

            /**
             * Set the forward (x) multiplier for the ticks reported by the forward deadwheels.
             *
             * @param forwardMul the multiplier
             * @return The builder
             */
            public Builder setXMultiplier(double forwardMul) {
                trackingWheelCoefficients.X_MULTIPLIER = forwardMul;
                return this;
            }

            /**
             * Set the strafe (y) multiplier for the ticks reported by the side deadwheel.
             *
             * @param strafeMul the multiplier
             * @return The builder
             */
            public Builder setYMultiplier(double strafeMul) {
                trackingWheelCoefficients.Y_MULTIPLIER = strafeMul;
                return this;
            }

            /**
             * Set the rate at which the IMU will set the pose estimate heading to the IMU reading for
             * relocalization.
             * <p>
             * This must be combined with a {@link #withRelocalizingIMU(IMU)} call on the ThreeWheelLocalizer if active.
             *
             * @param interval the interval time. zero/negative time intervals will continuously set the heading,
             *                 null will disable IMU interaction (default)
             * @return The builder
             */
            public Builder setImuRelocalizationInterval(@Nullable Measure<Time> interval) {
                trackingWheelCoefficients.IMU_RELOCALIZATION_INTERVAL = interval;
                return this;
            }

            /**
             * Build the coefficients.
             *
             * @return The coefficients
             */
            public ThreeWheelLocalizer.Coefficients build() {
                return trackingWheelCoefficients;
            }
        }
    }
}
