package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.intothedeep;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Centimeters;

import android.graphics.Color;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Neutral (yellow) sample processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 6.0.0
 */
@SuppressWarnings("MissingJavadoc")
public class NeutralSample extends ColourThreshold {
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(0, 150, 0);
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255, 255, 90);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Create a new Neutral Sample detector.
     */
    public NeutralSample() {
        setColourSpace(ColourSpace.YCrCb);
        setContourAreaMinPercent(() -> MIN_AREA);
        setContourAreaMaxPercent(() -> MAX_AREA);
        setLowerThreshold(() -> LOWER_YCRCB);
        setUpperThreshold(() -> UPPER_YCRCB);
        setBoxColour(Color.YELLOW);
        setShowMaskedInput(() -> SHOW_MASKED_INPUT);
        setErodeSize(3.5);
        setDilateSize(3.5);
        setBlurSize(3);
        setExternalContoursOnly(true);
        setPnP(Centimeters.of(9), Centimeters.of(3.5));
    }

    @Override
    @NonNull
    public String getId() {
        return "yellowsample";
    }
}
