package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Hook;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ColourBlob;

/**
 * BunyipsLib vision pipeline wrapper for a {@link ColorBlobLocatorProcessor}.
 * <p>
 * The {@link ColorBlobLocatorProcessor} finds "blobs" of a user-specified color
 * in the image. You can restrict the search area to a specified Region
 * of Interest (ROI).
 * <p>
 * This processor is the SDK-provided alternative to the {@link ColourThreshold} processor. Compared to
 * {@link ColourThreshold}, this processor is less advanced in terms of the data it provides, but does not
 * require a new class to be extended for each new colour range.
 *
 * @author Lucas Bubner, 2024
 * @see ColourSensor
 * @since 5.0.0
 */
public class ColourLocator extends Processor<ColourBlob> {
    private static int instances = 0;

    private final int instanceCount = instances++;
    private final ColorBlobLocatorProcessor instance;
    private volatile Object ctx;

    /**
     * Construct a new ColourLocator with minimum settings.
     *
     * @param regionOfInterest the area of interest on which to perform blob detection
     * @param targetRange      the colour range to filter for
     * @param generationMode   the type of data that will be reported by this processor
     */
    public ColourLocator(@NonNull ImageRegion regionOfInterest, @NonNull ColorRange targetRange, @NonNull ColorBlobLocatorProcessor.ContourMode generationMode) {
        instance = makeBuilderWithCommonSettings()
                .setRoi(regionOfInterest)
                .setTargetColorRange(targetRange)
                .setContourMode(generationMode)
                .build();
    }

    /**
     * Construct a new ColourLocator with extra and required settings supplied at your own discretion.
     *
     * @param builder the builder which you will supply your own colour blob locator settings
     */
    public ColourLocator(@NonNull Function<ColorBlobLocatorProcessor.Builder, ColorBlobLocatorProcessor.Builder> builder) {
        instance = builder.apply(makeBuilderWithCommonSettings()).build();
    }

    @Hook(on = Hook.Target.POST_STOP)
    private static void reset() {
        instances = 0;
    }

    private ColorBlobLocatorProcessor.Builder makeBuilderWithCommonSettings() {
        return new ColorBlobLocatorProcessor.Builder()
                .setDrawContours(true);
    }

    /**
     * Get the {@link ColorBlobLocatorProcessor} instance.
     *
     * @return direct wrapped instance from the SDK
     */
    @NonNull
    public ColorBlobLocatorProcessor getInstance() {
        return instance;
    }

    /**
     * Add a filter.
     *
     * @param filter the filter to add
     */
    public void addFilter(@NonNull ColorBlobLocatorProcessor.BlobFilter filter) {
        instance.addFilter(filter);
    }

    /**
     * Remove a filter.
     *
     * @param filter the filter to remove
     */
    public void removeFilter(@NonNull ColorBlobLocatorProcessor.BlobFilter filter) {
        instance.removeFilter(filter);
    }

    /**
     * Remove all filters.
     */
    public void removeAllFilters() {
        instance.removeAllFilters();
    }

    /**
     * Sets the list sorting method to use in the data.
     *
     * @param sortingMethod the sorting method to sort the array data
     */
    public void setSort(@NonNull ColorBlobLocatorProcessor.BlobSort sortingMethod) {
        instance.setSort(sortingMethod);
    }

    @NonNull
    @Override
    public String getId() {
        // 0-indexed
        return "colourlocator" + instanceCount;
    }

    @Override
    protected void update() {
        List<ColorBlobLocatorProcessor.Blob> blobs = instance.getBlobs();
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            // Need to wrap for consistency between vision processors,
            // will not bother with unboxing as it has been a bad idea in the past
            data.add(new ColourBlob(
                    // Camera dimensions are provided optionally and are handled internally only for conversion
                    // to a ContourData object, which means we don't need to check for null either
                    getCameraDimensions(),
                    blob.getContour(),
                    Arrays.asList(blob.getContourPoints()),
                    blob.getContourArea(),
                    blob.getDensity(),
                    blob.getAspectRatio(),
                    blob.getBoxFit()
            ));
        }
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
