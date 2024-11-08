package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * Colour thresholding processor for a colour space, used to find colour contours in an image and to use PnP
 * operations.
 * <p>
 * This processor was used initially pre-SDK 10.1 to detect contours, and now serves as an alternative option which offers
 * features like PnP and FtcDashboard tuning compared to the {@link ColourLocator}.
 *
 * @author Lucas Bubner, 2024
 * @see ColourLocator
 * @see ColourSensor
 * @since 6.0.0
 */
public abstract class ColourThreshold extends Processor<ContourData> {
    /**
     * The thickness of the border to draw around the biggest contour.
     */
    public static int DEFAULT_BIGGEST_CONTOUR_BORDER_THICKNESS = 6;
    /**
     * The thickness of the border to draw around all contours.
     */
    public static int DEFAULT_CONTOUR_BORDER_THICKNESS = 3;
    /**
     * Minimum contour area in percentage to threshold by.
     */
    public static double DEFAULT_MIN_AREA = 1.2;
    /**
     * Maximum contour area in percentage to threshold by.
     */
    public static double DEFAULT_MAX_AREA = 20.0;
    /**
     * Default box colour.
     */
    @ColorInt
    public static int DEFAULT_BOX_COLOUR = Color.WHITE;
    private final Mat processingMat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();
    // Required
    /**
     * The currently used Colour Space to use for lower and upper thresholds.
     */
    public Supplier<ColourSpace> colourSpace;
    /**
     * The currently used lower threshold to apply to filter out contours.
     */
    public Supplier<Scalar> lowerThreshold;
    /**
     * The currently used upper threshold to apply to filter out contours.
     */
    public Supplier<Scalar> upperThreshold;
    // Optional preferences
    private IntSupplier boxColour = () -> DEFAULT_BOX_COLOUR;
    private BooleanSupplier externalContoursOnly = () -> false;
    private DoubleSupplier minAreaPercent = () -> DEFAULT_MIN_AREA;
    private DoubleSupplier maxAreaPercent = () -> DEFAULT_MAX_AREA;
    private IntSupplier contourBorderThickness = () -> DEFAULT_CONTOUR_BORDER_THICKNESS;
    private IntSupplier biggestContourBorderThickness = () -> DEFAULT_BIGGEST_CONTOUR_BORDER_THICKNESS;
    private BooleanSupplier shouldShowMaskedInput = () -> true;
    // Optional processes
    private IntSupplier erodeSize = () -> 0;
    private IntSupplier dilateSize = () -> 0;
    private IntSupplier blurSizeUnnormalised = () -> 0;
    private int lastErodeSize, lastDilateSize, lastBlurSizeUnnormalised;
    private Mat erodeElement;
    private Mat dilateElement;
    private org.opencv.core.Size blurElement;

    // TODO: pnp

    /**
     * Creates a new ColourThreshold and auto-adds this class to FtcDashboard.
     */
    @SuppressWarnings("ConstructorNotProtectedInAbstractClass")
    public ColourThreshold() {
        Dashboard.enableConfig(getClass());
    }

    /**
     * Sets the colour space to use for this ColourThreshold.
     * This is a required method to call.
     *
     * @param colourSpace the colour space to use
     */
    public void setColourSpace(@NonNull Supplier<ColourSpace> colourSpace) {
        this.colourSpace = colourSpace;
    }

    /**
     * Sets the colour space to use for this ColourThreshold.
     * This is a required method to call.
     *
     * @param staticColourSpace the static colour space to use
     */
    public void setColourSpace(@NonNull ColourSpace staticColourSpace) {
        colourSpace = () -> staticColourSpace;
    }

    /**
     * Sets the upper scalar to use for thresholding.
     * This is a required method to call.
     *
     * @param upper the upper threshold in the colour space
     */
    public void setUpperThreshold(@NonNull Supplier<Scalar> upper) {
        upperThreshold = upper;
    }

    /**
     * Sets the upper scalar to use for thresholding.
     * This is a required method to call.
     *
     * @param staticUpper the static upper threshold in the colour space
     */
    public void setUpperThreshold(@NonNull Scalar staticUpper) {
        upperThreshold = () -> staticUpper;
    }

    /**
     * Sets the lower scalar to use for thresholding.
     * This is a required method to call.
     *
     * @param lower the lower threshold in the colour space
     */
    public void setLowerThreshold(@NonNull Supplier<Scalar> lower) {
        lowerThreshold = lower;
    }

    /**
     * Sets the lower scalar to use for thresholding.
     * This is a required method to call.
     *
     * @param staticLower the static lower threshold in the colour space
     */
    public void setLowerThreshold(@NonNull Scalar staticLower) {
        lowerThreshold = () -> staticLower;
    }

    /**
     * Sets the box colour to use for highlighting contours.
     * This is an optional method.
     *
     * @param colour the box colour to use, default WHITE
     */
    public void setBoxColour(@NonNull IntSupplier colour) {
        boxColour = colour;
    }

    /**
     * Sets the box colour to use for highlighting contours.
     * This is an optional method.
     *
     * @param staticColour the static box colour to use, default WHITE
     */
    public void setBoxColour(@ColorInt int staticColour) {
        boxColour = () -> staticColour;
    }

    /**
     * Set the lower percentage that will be used as a mininum.
     * This is an optional method.
     *
     * @param percent the minimum percentage of the screen the contour should be to register, default 1.2%
     */
    public void setContourAreaMinPercent(@NonNull DoubleSupplier percent) {
        minAreaPercent = percent;
    }

    /**
     * Set the lower percentage that will be used as a mininum.
     * This is an optional method.
     *
     * @param staticPercent the static minimum percentage of the screen the contour should be to register, default 1.2%
     */
    public void setContourAreaMinPercent(double staticPercent) {
        minAreaPercent = () -> staticPercent;
    }

    /**
     * Set the upper percentage that will be used as a maximum.
     * This is an optional method.
     *
     * @param percent the maximum percentage of the screen can be to be registered, default 20%
     */
    public void setContourAreaMaxPercent(@NonNull DoubleSupplier percent) {
        maxAreaPercent = percent;
    }

    /**
     * Set the upper percentage that will be used as a maximum.
     * This is an optional method.
     *
     * @param staticPercent the static maximum percentage of the screen can be to be registered, default 20%
     */
    public void setContourAreaMaxPercent(double staticPercent) {
        maxAreaPercent = () -> staticPercent;
    }

    /**
     * Whether to show the colour thresholding on the output mat.
     * This is an optional method.
     *
     * @param showMaskedInput whether to show the thresholding on the output mat, default true
     */
    public void setShowMaskedInput(@NonNull BooleanSupplier showMaskedInput) {
        shouldShowMaskedInput = showMaskedInput;
    }

    /**
     * Whether to show the colour thresholding on the output mat.
     * This is an optional method.
     *
     * @param staticShowMaskedInput whether to show the thresholding on the output mat, default true, static option
     */
    public void setShowMaskedInput(boolean staticShowMaskedInput) {
        shouldShowMaskedInput = () -> staticShowMaskedInput;
    }

    /**
     * Sets the border thickness of a detected contour that is not the largest in the scene.
     * This is an optional method.
     *
     * @param thicknessPx thickness in pixels, default 3px
     */
    public void setContourBorderThickness(@NonNull IntSupplier thicknessPx) {
        contourBorderThickness = thicknessPx;
    }

    /**
     * Sets the border thickness of a detected contour that is not the largest in the scene.
     * This is an optional method.
     *
     * @param staticThicknessPx static thickness in pixels, default 3px
     */
    public void setContourBorderThickness(int staticThicknessPx) {
        contourBorderThickness = () -> staticThicknessPx;
    }

    /**
     * Sets the border thickness of a detected contour that is the largest in the scene.
     * This is an optional method.
     *
     * @param thicknessPx thickness in pixels, default 6px
     */
    public void setBiggestContourBorderThickness(@NonNull IntSupplier thicknessPx) {
        biggestContourBorderThickness = thicknessPx;
    }

    /**
     * Sets the border thickness of a detected contour that is the largest in the scene.
     * This is an optional method.
     *
     * @param staticThicknessPx static thickness in pixels, default 6px
     */
    public void setBiggestContourBorderThickness(int staticThicknessPx) {
        biggestContourBorderThickness = () -> staticThicknessPx;
    }

    /**
     * Filter out contours that are inside other contours.
     * This is an optional method.
     *
     * @param useExternalContours whether to only scan for contours that are not nested in other contours, default false
     */
    public void setExternalContoursOnly(@NonNull BooleanSupplier useExternalContours) {
        externalContoursOnly = useExternalContours;
    }

    /**
     * Filter out contours that are inside other contours.
     * This is an optional method.
     *
     * @param staticUseExternalContours whether to only scan for contours that are not nested in other contours, default false, static option
     */
    public void setExternalContoursOnly(boolean staticUseExternalContours) {
        externalContoursOnly = () -> staticUseExternalContours;
    }

    /**
     * Sets the size of the eroding element to use during thresholding.
     * Erosion eats away at the mask, reducing noise by eliminating super small areas, but also reduces the
     * contour areas of everything a little bit.
     * This is an optional method.
     *
     * @param erodeSize size of the erode element in pixels, default/<=0 is disabled
     */
    public void setErodeSize(@NonNull IntSupplier erodeSize) {
        this.erodeSize = erodeSize;
    }

    /**
     * Sets the size of the eroding element to use during thresholding.
     * Erosion eats away at the mask, reducing noise by eliminating super small areas, but also reduces the
     * contour areas of everything a little bit.
     * This is an optional method.
     *
     * @param staticErodeSize static size of the erode element in pixels, default/<=0 is disabled
     */
    public void setErodeSize(int staticErodeSize) {
        erodeSize = () -> staticErodeSize;
    }

    /**
     * Sets the size of the dilating element to use during thresholding.
     * Dilation expands mask areas, making up for shrinkage caused during erosion, and can also clean up results
     * by closing small interior gaps in the mask.
     * This is an optional method.
     *
     * @param dilateSize size of the dilate element in pixels, default/<=0 is disabled
     */
    public void setDilateSize(@NonNull IntSupplier dilateSize) {
        this.dilateSize = dilateSize;
    }

    /**
     * Sets the size of the dilating element to use during thresholding.
     * Dilation expands mask areas, making up for shrinkage caused during erosion, and can also clean up results
     * by closing small interior gaps in the mask.
     * This is an optional method.
     *
     * @param staticDilateSize static size of the dilate element in pixels, default/<=0 is disabled
     */
    public void setDilateSize(int staticDilateSize) {
        dilateSize = () -> staticDilateSize;
    }

    /**
     * Sets the size of the blur element to use during thresholding.
     * Blurring can improve color thresholding results by smoothing color variation.
     * This is an optional method.
     *
     * @param blurSizeUnnormalised the blur size that will be renormalised later to be an odd number, default/<=0 is disabled
     */
    public void setBlurSize(@NonNull IntSupplier blurSizeUnnormalised) {
        this.blurSizeUnnormalised = blurSizeUnnormalised;
    }

    /**
     * Sets the size of the blur element to use during thresholding.
     * Blurring can improve color thresholding results by smoothing color variation.
     * This is an optional method.
     *
     * @param staticBlurSizeUnnormalised the static blur size that will be renormalised later to be an odd number, default/<=0 is disabled
     */
    public void setBlurSize(int staticBlurSizeUnnormalised) {
        blurSizeUnnormalised = () -> staticBlurSizeUnnormalised;
    }

    private void reinitialiseMats() {
        int erode = erodeSize.getAsInt();
        int dilate = dilateSize.getAsInt();
        int blur = blurSizeUnnormalised.getAsInt();

        erodeElement = erode > 0
                ? Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(erode, erode))
                : null;
        dilateElement = dilate > 0
                ? Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(dilate, dilate))
                : null;
        blurElement = blur > 0
                ? new org.opencv.core.Size(blur | 0x01, blur | 0x01)
                : null;

        lastErodeSize = erode;
        lastDilateSize = dilate;
        lastBlurSizeUnnormalised = blur;
    }

    @Override
    public void init(int width, int height, @Nullable CameraCalibration calibration) {
        reinitialiseMats();
    }

    @Override
    protected final void update() {
        Size cameraDimensions = getCameraDimensions();
        if (cameraDimensions == null) return;
        for (MatOfPoint contour : contours) {
            Point[] points = contour.toArray();
            ContourData newData = new ContourData(cameraDimensions, Imgproc.minAreaRect(new MatOfPoint2f(points)));
            // Min-max bounding
            double min = Mathf.clamp(minAreaPercent.getAsDouble(), 0, 100);
            double max = Mathf.clamp(maxAreaPercent.getAsDouble(), 0, 100);
            if (newData.getAreaPercent() < min || newData.getAreaPercent() > max)
                continue;
            data.add(newData);
        }
    }

    @Override
    protected final void onProcessFrame(@NonNull Mat frame, long captureTimeNanos) {
        if (colourSpace == null || lowerThreshold == null || upperThreshold == null) {
            throw new IllegalStateException("This colour threshold has not been configured properly! Ensure the colour space, lower and upper thresholds have been set using the `set` methods.");
        }

        if (erodeSize.getAsInt() != lastErodeSize
                || dilateSize.getAsInt() != lastDilateSize
                || blurSizeUnnormalised.getAsInt() != lastBlurSizeUnnormalised) {
            reinitialiseMats();
        }

        /*
         * Converts our input mat from RGB to
         * specified color space by the enum.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "processingMat"
         */
        Imgproc.cvtColor(frame, processingMat, colourSpace.get().cvtCode);

        if (blurElement != null) {
            Imgproc.GaussianBlur(processingMat, processingMat, blurElement, 0);
        }

        /*
         * This is where our thresholding actually happens.
         * Takes our "processingMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars "lower" and "upper".
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(processingMat, lowerThreshold.get(), upperThreshold.get(), binaryMat);

        if (erodeElement != null) {
            Imgproc.erode(binaryMat, binaryMat, erodeElement);
        }

        if (dilateElement != null) {
            Imgproc.dilate(binaryMat, binaryMat, dilateElement);
        }

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        /*
         * Find the contours of the binary Mat. This will
         * populate the "contours" list with the contours
         * found in the binary Mat.
         */
        contours.clear();
        Imgproc.findContours(
                binaryMat,
                contours,
                hierarchy,
                externalContoursOnly.getAsBoolean() ? Imgproc.RETR_EXTERNAL : Imgproc.RETR_LIST,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // Only show the detection matrix if we need to
        if (shouldShowMaskedInput.getAsBoolean())
            maskedInputMat.copyTo(frame);

        binaryMat.release();
        processingMat.release();
        hierarchy.release();
    }

    @Override
    protected final void onFrameDraw(@NonNull Canvas canvas) {
        // Draw borders around the contours, with a thicker border for the largest contour
        ContourData biggest = ContourData.getLargest(data);
        for (ContourData contour : data) {
            Point[] rotRectPts = new Point[4];
            contour.getRect().points(rotRectPts);
            for (int i = 0; i < 4; i++) {
                canvas.drawLine(
                        (float) rotRectPts[i].x, (float) rotRectPts[i].y,
                        (float) rotRectPts[(i + 1) % 4].x, (float) rotRectPts[(i + 1) % 4].y,
                        new Paint() {{
                            setColor(boxColour.getAsInt());
                            setStyle(Style.STROKE);
                            setStrokeWidth(contour == biggest ? DEFAULT_BIGGEST_CONTOUR_BORDER_THICKNESS : DEFAULT_CONTOUR_BORDER_THICKNESS);
                        }}
                );
            }
        }
    }

    /**
     * The colour spaces that can be used for thresholding.
     */
    public enum ColourSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */

        /**
         * Red, Green, Blue.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        /**
         * Hue, Saturation, Value.
         */
        HSV(Imgproc.COLOR_RGB2HSV),
        /**
         * Luminance, Chrominance.
         */
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        /**
         * Lightness, A, B.
         */
        Lab(Imgproc.COLOR_RGB2Lab);

        /**
         * OpenCV conversion code for the colour space.
         */
        public final int cvtCode;

        ColourSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }

        /**
         * Get a component (channel) name from the colour space at a given index (0-2).
         *
         * @param idx the index of the channel, 0-2
         * @return the name of the channel, or null if the index is out of bounds
         */
        @NonNull
        public final String getChannelName(int idx) {
            switch (this) {
                case RGB:
                    return new String[]{"Red", "Green", "Blue"}[idx];
                case HSV:
                    return new String[]{"Hue", "Saturation", "Value"}[idx];
                case YCrCb:
                    return new String[]{"Luminance", "Chrominance Red", "Chrominance Blue"}[idx];
                case Lab:
                    return new String[]{"Lightness", "A", "B"}[idx];
                default:
                    return "Unknown";
            }
        }
    }
}
