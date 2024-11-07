package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Purple pixel processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 1.0.0-pre
 */
public class PurplePixel extends ColourThreshold {
    /**
     * Lower bounds for the YCrCb colour space.
     */
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(150, 0.0, 145.8);
    /**
     * Upper bounds for the YCrCb colour space.
     */
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255.0, 255.0, 255.0);
    /**
     * Default minimum area for the contour.
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default maximum area for the contour.
     */
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    /**
     * Whether to show the masked input on the screen.
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using the YCrCb colour space.
     */
    public PurplePixel() {
        super(ColourSpace.YCrCb);
        Dashboard.enableConfig(getClass());
    }

    @NonNull
    @Override
    public String toString() {
        return "purplepixel";
    }

    @Override
    public double getContourAreaMinPercent() {
        return MIN_AREA;
    }

    @Override
    public double getContourAreaMaxPercent() {
        return MAX_AREA;
    }

    @NonNull
    @Override
    protected Scalar getLowerThreshold() {
        return LOWER_YCRCB;
    }

    @NonNull
    @Override
    protected Scalar getUpperThreshold() {
        return UPPER_YCRCB;
    }

    @Override
    public int getBoxColour() {
        return 0xFF800080;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }
}
