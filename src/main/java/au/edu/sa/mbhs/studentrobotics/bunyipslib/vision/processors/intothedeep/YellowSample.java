package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.intothedeep;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Yellow sample processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 6.0.0
 */
@SuppressWarnings("MissingJavadoc")
public class YellowSample extends ColourThreshold {
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(0, 130, 0);
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255, 255, 255);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Create a new Yellow Sample detector.
     */
    public YellowSample() {
        setColourSpace(ColourSpace.YCrCb);
        setContourAreaMinPercent(() -> MIN_AREA);
        setContourAreaMaxPercent(() -> MAX_AREA);
        setLowerThreshold(() -> LOWER_YCRCB);
        setUpperThreshold(() -> UPPER_YCRCB);
        setBoxColour(0xFFFFFF00);
        setShowMaskedInput(() -> SHOW_MASKED_INPUT);
    }

    @Override
    @NonNull
    public String getId() {
        return "yellowsample";
    }
}
