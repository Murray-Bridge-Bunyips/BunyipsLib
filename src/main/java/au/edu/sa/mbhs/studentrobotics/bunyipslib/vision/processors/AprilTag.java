package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

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
    public void init(int width, int height, @NonNull CameraCalibration calibration) {
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
}
