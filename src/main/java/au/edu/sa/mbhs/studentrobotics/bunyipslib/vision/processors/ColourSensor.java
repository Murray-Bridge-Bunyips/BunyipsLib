package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Mat;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hooks.Hook;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ColourSample;

/**
 * BunyipsLib vision pipeline wrapper for a {@link PredominantColorProcessor}.
 * <p>
 * The {@link PredominantColorProcessor} acts like a "Color Sensor",
 * allowing you to define a Region of Interest (ROI) of the camera
 * stream inside which the dominant color is found. Additionally,
 * said color is matched to one of the {@link PredominantColorProcessor.Swatch}s specified by
 * the user as a "best guess" at the general shade of the color.
 *
 * @author Lucas Bubner, 2024
 * @see ColourLocator
 * @since 5.0.0
 */
public class ColourSensor extends Processor<ColourSample> {
    private static int instances = 0;

    private final int instanceCount = instances++;
    private final PredominantColorProcessor instance;
    private volatile Object ctx;

    /**
     * Construct a new ColourSensor vision pipeline.
     *
     * @param regionOfInterest the region of interest on which to perform colour analysis
     * @param swatches         the swatches from which a "best guess" at the shade of the predominant colour will be made
     */
    public ColourSensor(@NonNull ImageRegion regionOfInterest, @NonNull PredominantColorProcessor.Swatch... swatches) {
        instance = new PredominantColorProcessor.Builder()
                .setRoi(regionOfInterest)
                .setSwatches(swatches)
                .build();
    }

    @Hook(on = Hook.Target.POST_STOP)
    private static void reset() {
        instances = 0;
    }

    /**
     * Get the {@link PredominantColorProcessor} instance.
     *
     * @return direct wrapped instance from the SDK
     */
    @NonNull
    public PredominantColorProcessor getInstance() {
        return instance;
    }

    @NonNull
    @Override
    public String getId() {
        // 0-indexed
        return "coloursensor" + instanceCount;
    }

    @Override
    protected void update() {
        PredominantColorProcessor.Result res = instance.getAnalysis();
        // Guaranteed to be only one colour sample instance in the data object at all times
        data.add(new ColourSample(res.closestSwatch, res.rgb));
    }

    @Override
    public void init(@Nullable CameraCalibration calibration) {
        Size cameraDimensions = getCameraDimensions();
        assert cameraDimensions != null;
        int width = cameraDimensions.getWidth();
        int height = cameraDimensions.getHeight();
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
