package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.centerstage;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.ColourThreshold;

/**
 * Utility construction for pixel detectors.
 *
 * @since 1.0.0-pre
 */
public final class Pixels {
    private Pixels() {
    }

    /**
     * Create all the pixel processors.
     *
     * @return An array of all the pixel processors - White Pixel, Purple Pixel, Yellow Pixel, and Green Pixel
     */
    @NonNull
    public static ColourThreshold[] createProcessors() {
        return new ColourThreshold[]{
                new WhitePixel(),
                new PurplePixel(),
                new YellowPixel(),
                new GreenPixel()
        };
    }
}
