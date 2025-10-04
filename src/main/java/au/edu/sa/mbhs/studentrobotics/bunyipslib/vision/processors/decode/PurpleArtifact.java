package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.decode;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Purple artifact processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 8.0.0
 */
@SuppressWarnings("MissingJavadoc")
public class PurpleArtifact extends ColourThreshold {
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(32, 135, 135);
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255, 155, 169);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Create a new Purple Artifact detector.
     */
    public PurpleArtifact() {
        setColourSpace(ColourSpace.YCrCb);
        setContourAreaMinPercent(() -> MIN_AREA);
        setContourAreaMaxPercent(() -> MAX_AREA);
        setLowerThreshold(() -> LOWER_YCRCB);
        setUpperThreshold(() -> UPPER_YCRCB);
        setBoxColour(0xFF800080);
        setShowMaskedInput(() -> SHOW_MASKED_INPUT);
        setExternalContoursOnly(true);
    }

    @Override
    @NonNull
    public String getId() {
        return "purpleartifact";
    }
}
