package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.ColourBlob;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

/**
 * BunyipsLib vision pipeline wrapper for a {@link ColorBlobLocatorProcessor}.
 * <p>
 * The {@link ColorBlobLocatorProcessor} finds "blobs" of a user-specified color
 * in the image. You can restrict the search area to a specified Region
 * of Interest (ROI).
 * <p>
 * This processor is the SDK-provided successor to the {@link ColourThreshold} processor.
 *
 * @author Lucas Bubner, 2024
 * @see ColourSensor
 * @since 5.0.0
 */
public class ColourLocator extends Processor<ColourBlob> {
    private final ColorBlobLocatorProcessor instance;
    private volatile Object ctx;

    /**
     * Construct a new ColourLocator with minimum settings.
     *
     * @param targetRange    the colour range to filter for
     * @param generationMode the type of data that will be reported by this processor
     */
    public ColourLocator(ColorRange targetRange, ColorBlobLocatorProcessor.ContourMode generationMode) {
        instance = makeBuilderWithCommonSettings()
                .setTargetColorRange(targetRange)
                .setContourMode(generationMode)
                .build();
    }

    /**
     * Construct a new ColourLocator with extra and required settings supplied at your own discretion.
     *
     * @param builder the builder which you will supply your own colour blob locator settings
     */
    public ColourLocator(Function<ColorBlobLocatorProcessor.Builder, ColorBlobLocatorProcessor.Builder> builder) {
        instance = builder.apply(makeBuilderWithCommonSettings()).build();
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
    public ColorBlobLocatorProcessor getInstance() {
        return instance;
    }

    /**
     * Add a filter.
     *
     * @param filter the filter to add
     */
    public void addFilter(ColorBlobLocatorProcessor.BlobFilter filter) {
        instance.addFilter(filter);
    }

    /**
     * Remove a filter.
     *
     * @param filter the filter to remove
     */
    public void removeFilter(ColorBlobLocatorProcessor.BlobFilter filter) {
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
    public void setSort(ColorBlobLocatorProcessor.BlobSort sortingMethod) {
        instance.setSort(sortingMethod);
    }

    @NonNull
    @Override
    public String toString() {
        return "colourlocator";
    }

    @Override
    protected void update() {
        List<ColorBlobLocatorProcessor.Blob> blobs = instance.getBlobs();
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            // Need to wrap for consistency between vision processors,
            // will not bother with unboxing as it has been a bad idea in the past
            data.add(new ColourBlob(
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
        if (dimensions == null) return;
        instance.onDrawFrame(canvas, dimensions.getWidth(), dimensions.getHeight(), 1.0f, 1.0f, ctx);
    }
}
