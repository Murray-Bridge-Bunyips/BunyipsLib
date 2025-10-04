package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.decode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Green artifact processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 8.0.0
 */
@SuppressWarnings("MissingJavadoc")
public class GreenArtifact extends ColourThreshold {
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(32, 50, 118);
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255, 105, 145);
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Create a new Green Artifact detector.
     */
    public GreenArtifact() {
        setColourSpace(ColourSpace.YCrCb);
        setContourAreaMinPercent(() -> MIN_AREA);
        setContourAreaMaxPercent(() -> MAX_AREA);
        setLowerThreshold(() -> LOWER_YCRCB);
        setUpperThreshold(() -> UPPER_YCRCB);
        setBoxColour(Color.GREEN);
        setShowMaskedInput(() -> SHOW_MASKED_INPUT);
        setExternalContoursOnly(true);
    }

    @Override
    @NonNull
    public String getId() {
        return "greenartifact";
    }
}
