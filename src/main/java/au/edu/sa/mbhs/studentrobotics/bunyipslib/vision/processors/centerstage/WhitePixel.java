package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * White pixel processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 1.0.0-pre
 */
@SuppressWarnings("MissingJavadoc")
public class WhitePixel extends ColourThreshold {
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(250, 120.0, 106.3);
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255.0, 129.0, 255.0);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Create a new White Pixel detector.
     */
    public WhitePixel() {
        setColourSpace(ColourSpace.YCrCb);
        setContourAreaMinPercent(() -> MIN_AREA);
        setContourAreaMaxPercent(() -> MAX_AREA);
        setLowerThreshold(() -> LOWER_YCRCB);
        setUpperThreshold(() -> UPPER_YCRCB);
        setBoxColour(0xFFFFFFFF);
        setShowMaskedInput(() -> SHOW_MASKED_INPUT);
    }

    @NonNull
    @Override
    public String getId() {
        return "whitepixel";
    }
}
