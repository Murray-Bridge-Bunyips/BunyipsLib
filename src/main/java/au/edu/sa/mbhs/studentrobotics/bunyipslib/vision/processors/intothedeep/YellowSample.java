package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.intothedeep;

import androidx.annotation.NonNull;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Yellow Sample
 */
public class YellowSample extends ColourThreshold {
    /**
     * Lower clamp for YCrCb
     */
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(0, 130, 0);
    /**
     * Upper clamp for YCrCb
     */
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255, 255, 255);
    /**
     * Default minimum area for the contour
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default maximum area for the contour
     */
    public static double MAX_AREA = 20;
    /**
     * Whether to show the masked input
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using YCrCb colour space
     */
    public YellowSample() {
        // TODO: refactor colourthreshold and integrate pnp
        super(ColourSpace.YCrCb);
        Dashboard.enableConfig(getClass());
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
        return 0xFFFFFF00;
    }

    @Override
    public boolean showMaskedInput() {
        return SHOW_MASKED_INPUT;
    }

    @Override
    @NonNull
    public String toString() {
        return "yellowsample";
    }
}
