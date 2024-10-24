package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;

import org.opencv.core.Scalar;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Yellow pixel processor.
 * These values may not be tuned for your specific camera, lighting, or field conditions, and are tuned
 * based on our own testing. You may need to adjust these values to get the best results for your own robot.
 *
 * @since 1.0.0-pre
 */
public class YellowPixel extends ColourThreshold {
    /**
     * Lower bounds for YCrCb
     */
    @NonNull
    public static Scalar LOWER_YCRCB = new Scalar(0.0, 150.0, 0.0);
    /**
     * Upper bounds for YCrCb
     */
    @NonNull
    public static Scalar UPPER_YCRCB = new Scalar(255.0, 255.0, 82.2);
    /**
     * Default min area for detections.
     */
    public static double MIN_AREA = DEFAULT_MIN_AREA;
    /**
     * Default max area for detections.
     */
    public static double MAX_AREA = DEFAULT_MAX_AREA;
    /**
     * Whether to show the masked input on the screen.
     */
    public static boolean SHOW_MASKED_INPUT = true;

    /**
     * Using YCrCb colour space.
     */
    public YellowPixel() {
        super(ColourSpace.YCrCb);
        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    @NonNull
    @Override
    public String toString() {
        return "yellowpixel";
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
    protected Scalar setLower() {
        return LOWER_YCRCB;
    }

    @NonNull
    @Override
    protected Scalar setUpper() {
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
}
