package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Mat;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ColourSample;

/**
 * BunyipsLib vision pipeline wrapper for a {@link PredominantColorProcessor}.
 * <p>
 * The {@link PredominantColorProcessor} acts like a "Color Sensor",
 * allowing you to define a Region of Interest (ROI) of the camera
 * stream inside of which the dominant color is found. Additionally,
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
    public ColourSensor(ImageRegion regionOfInterest, PredominantColorProcessor.Swatch... swatches) {
        instance = new PredominantColorProcessor.Builder()
                .setRoi(regionOfInterest)
                .setSwatches(swatches)
                .build();
    }

    /**
     * Reset instance count for processor identification.
     */
    public static void resetForOpMode() {
        instances = 0;
    }

    /**
     * Get the {@link PredominantColorProcessor} instance.
     *
     * @return direct wrapped instance from the SDK
     */
    public PredominantColorProcessor getInstance() {
        return instance;
    }

    @NonNull
    @Override
    public String toString() {
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
    public void init(int width, int height, CameraCalibration calibration) {
        instance.init(width, height, calibration);
    }

    @Override
    protected void onProcessFrame(Mat frame, long captureTimeNanos) {
        ctx = instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    protected void onFrameDraw(Canvas canvas) {
        Size dimensions = getCameraDimensions();
        if (dimensions == null || ctx == null) return;
        instance.onDrawFrame(canvas, dimensions.getWidth(), dimensions.getHeight(), 1.0f, 1.0f, ctx);
    }
}
