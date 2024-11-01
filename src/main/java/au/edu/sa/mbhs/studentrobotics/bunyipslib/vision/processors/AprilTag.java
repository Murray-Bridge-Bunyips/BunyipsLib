package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;

/**
 * AprilTag Detection Processor
 * <p>
 * This is an extension wrapper for the SDK-provided AprilTagProcessor to interop with the Vision system.
 *
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
public class AprilTag extends Processor<AprilTagData> {
    private final AprilTagProcessor instance;
    private volatile Object ctx;

    /**
     * Construct a new AprilTag processor.
     * Will rely on the SDK and the teamwebcamcalibrations.xml file to provide camera intrinsics for this overload.
     */
    public AprilTag() {
        instance = makeBuilderWithCommonSettings().build();
    }

    /**
     * Create an AprilTag instance with the additional builder settings as provided.
     * Will rely on the SDK and the teamwebcamcalibrations.xml file to provide camera intrinsics for this overload,
     * though you can set your own camera intrinsics using the builder settings.
     *
     * @param customBuilderSettings additional settings that will be attached to the builder
     */
    public AprilTag(@NonNull Function<AprilTagProcessor.Builder, AprilTagProcessor.Builder> customBuilderSettings) {
        instance = customBuilderSettings.apply(makeBuilderWithCommonSettings()).build();
    }

    /**
     * Creates a builder to assist in setting the camera pose for the AprilTagProcessor.
     *
     * @param builder the builder to apply the camera pose to
     * @return the builder to set the camera pose
     */
    public static AprilTagCameraPoseBuilder setCameraPose(AprilTagProcessor.Builder builder) {
        return new AprilTagCameraPoseBuilder(builder);
    }

    private AprilTagProcessor.Builder makeBuilderWithCommonSettings() {
        return new AprilTagProcessor.Builder()
                // Common (always enabled) settings
                .setDrawAxes(true)
                .setDrawCubeProjection(true);
    }

    /**
     * Get the AprilTag instance
     *
     * @return direct AprilTagProcessor instance
     */
    @NonNull
    public AprilTagProcessor getInstance() {
        return instance;
    }

    @NonNull
    @Override
    public String toString() {
        return "apriltag";
    }

    @Override
    protected void update() {
        List<AprilTagDetection> detections = instance.getDetections();
        for (AprilTagDetection detection : detections) {
            // Need to wrap the AprilTagDetection in an extension of VisionData for consistency
            data.add(new AprilTagData(
                    detection.id,
                    detection.hamming,
                    detection.decisionMargin,
                    detection.center,
                    Arrays.asList(detection.corners),
                    Optional.ofNullable(detection.metadata),
                    Optional.ofNullable(detection.ftcPose),
                    Optional.ofNullable(detection.robotPose),
                    Optional.ofNullable(detection.rawPose),
                    detection.frameAcquisitionNanoTime
            ));
        }
    }

    // Untouched methods to be handled by the AprilTagProcessor

    @Override
    public void init(int width, int height, @Nullable CameraCalibration calibration) {
        instance.init(width, height, calibration);
    }

    @Override
    protected void onProcessFrame(@NonNull Mat frame, long captureTimeNanos) {
        ctx = instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    protected void onFrameDraw(@NonNull Canvas canvas) {
        Size dimensions = getCameraDimensions();
        if (dimensions == null || ctx == null) return;
        instance.onDrawFrame(canvas, dimensions.getWidth(), dimensions.getHeight(), 1.0f, 1.0f, ctx);
    }

    /**
     * Utility for setting the camera pose for the AprilTagProcessor.
     */
    public static class AprilTagCameraPoseBuilder {
        private final AprilTagProcessor.Builder builder;
        private double forwardIn;
        private double rightIn;
        private double upIn;
        private double yawDeg;
        private double pitchDeg = -90;
        private double rollDeg;

        /**
         * Construct a new AprilTagCameraPoseBuilder
         *
         * @param builder the builder to apply the camera pose to
         */
        public AprilTagCameraPoseBuilder(AprilTagProcessor.Builder builder) {
            this.builder = builder;
        }

        /**
         * Set the position of the camera relative to the robot's center of rotation.
         *
         * @param position the raw {@link Position} instance to use, must follow the standard FTC coordinate system
         * @return this instance
         */
        public AprilTagCameraPoseBuilder withCameraPosition(Position position) {
            forwardIn = position.x;
            // noinspection SuspiciousNameCombination
            rightIn = position.y;
            upIn = position.z;
            return this;
        }

        /**
         * Set the position of the camera relative to the robot's center of rotation.
         * Defaults to the camera being directly above the robot's center of rotation at field height.
         *
         * @param forward the distance forward of the robot's center of rotation (+forward is forward)
         * @param left    the distance left of the robot's center of rotation (+left is left)
         * @param up      the distance above the robot's center of rotation at field height (+up is up)
         * @return this instance
         */
        public AprilTagCameraPoseBuilder withCameraPosition(Measure<Distance> forward, Measure<Distance> left, Measure<Distance> up) {
            forwardIn = forward.in(Inches);
            rightIn = left.negate().in(Inches);
            upIn = up.in(Inches);
            return this;
        }

        /**
         * Set the orientation of the camera relative to the robot's center of rotation.
         *
         * @param orientation the raw {@link YawPitchRollAngles} instance to use, must follow the standard FTC coordinate system
         * @return this instance
         */
        public AprilTagCameraPoseBuilder withCameraOrientation(YawPitchRollAngles orientation) {
            yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            pitchDeg = orientation.getPitch(AngleUnit.DEGREES);
            rollDeg = orientation.getRoll(AngleUnit.DEGREES);
            return this;
        }

        /**
         * Set the orientation of the camera relative to the robot's center of rotation.
         *
         * @param yaw   the yaw angle of the camera, zero defaults to forward, +yaw is left, -yaw is right
         * @param pitch the pitch angle of the camera, zero defaults to parallel with the ground, +pitch is up, -pitch is down
         * @param roll  the roll angle of the camera, zero defaults to no roll, +roll is clockwise, -roll is counterclockwise
         * @return this instance
         */
        public AprilTagCameraPoseBuilder withCameraOrientation(Measure<Angle> yaw, Measure<Angle> pitch, Measure<Angle> roll) {
            yawDeg = yaw.in(Degrees);
            pitchDeg = -90 + pitch.in(Degrees);
            rollDeg = roll.in(Degrees);
            return this;
        }

        /**
         * Set the forward distance of the camera relative to the robot's center of rotation.
         *
         * @param forward the distance forward of the robot's center of rotation (+forward is forward)
         * @return this instance
         */
        public AprilTagCameraPoseBuilder forward(Measure<Distance> forward) {
            forwardIn = forward.in(Inches);
            return this;
        }

        /**
         * Set the left distance of the camera relative to the robot's center of rotation.
         *
         * @param left the distance left of the robot's center of rotation (+left is left)
         * @return this instance
         */
        public AprilTagCameraPoseBuilder left(Measure<Distance> left) {
            rightIn = left.negate().in(Inches);
            return this;
        }

        /**
         * Set the up distance of the camera relative to the robot's center of rotation.
         *
         * @param up the distance above the robot's center of rotation at field height (+up is up)
         * @return this instance
         */
        public AprilTagCameraPoseBuilder up(Measure<Distance> up) {
            upIn = up.in(Inches);
            return this;
        }

        /**
         * Set the yaw angle of the camera relative to the robot's center of rotation.
         *
         * @param yaw the yaw angle of the camera, zero defaults to forward, +yaw is left, -yaw is right
         * @return this instance
         */
        public AprilTagCameraPoseBuilder yaw(Measure<Angle> yaw) {
            yawDeg = yaw.in(Degrees);
            return this;
        }

        /**
         * Set the pitch angle of the camera relative to the robot's center of rotation.
         *
         * @param pitch the pitch angle of the camera, zero defaults to parallel with the ground, +pitch is up, -pitch is down
         * @return this instance
         */
        public AprilTagCameraPoseBuilder pitch(Measure<Angle> pitch) {
            pitchDeg = -90 + pitch.in(Degrees);
            return this;
        }

        /**
         * Set the roll angle of the camera relative to the robot's center of rotation.
         *
         * @param roll the roll angle of the camera, zero defaults to no roll, +roll is clockwise, -roll is counterclockwise
         * @return this instance
         */
        public AprilTagCameraPoseBuilder roll(Measure<Angle> roll) {
            rollDeg = roll.in(Degrees);
            return this;
        }

        /**
         * Apply the configured camera pose to the builder.
         *
         * @return the original builder instance with the camera pose applied
         */
        public AprilTagProcessor.Builder apply() {
            builder.setCameraPose(
                    new Position(DistanceUnit.INCH, rightIn, forwardIn, upIn, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, yawDeg, pitchDeg, rollDeg, 0)
            );
            return builder;
        }
    }
}
