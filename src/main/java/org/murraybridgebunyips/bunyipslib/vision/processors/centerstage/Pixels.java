package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;

/**
 * Utility construction for pixel detectors.
 */
public class Pixels {
    private Pixels() {
    }

    /**
     * Create all the pixel processors.
     * @return An array of all the pixel processors - White Pixel, Purple Pixel, Yellow Pixel, and Green Pixel
     */
    public static YCbCrColourThreshold[] createProcessors() {
        return new YCbCrColourThreshold[]{
                new WhitePixel(),
                new PurplePixel(),
                new YellowPixel(),
                new GreenPixel()
        };
    }

    /**
     * Set the thresholds for a given Pixel processor.
     * @param processor The processor to set the thresholds for
     * @param yLower The lower Y threshold
     * @param cbLower The lower Cb threshold
     * @param crLower The lower Cr threshold
     * @param yUpper The upper Y threshold
     * @param cbUpper The upper Cb threshold
     * @param crUpper The upper Cr threshold
     */
    public static void setThresholds(YCbCrColourThreshold processor, double yLower, double cbLower, double crLower, double yUpper, double cbUpper, double crUpper) {
        // We sadly have to do it like this because we need static fields on every processor
        // in order to make them configurable in the dashboard
        if (processor instanceof WhitePixel) {
            WhitePixel.LOWER_Y = yLower;
            WhitePixel.UPPER_Y = yUpper;
            WhitePixel.LOWER_CR = crLower;
            WhitePixel.UPPER_CR = crUpper;
            WhitePixel.LOWER_CB = cbLower;
            WhitePixel.UPPER_CB = cbUpper;
        } else if (processor instanceof PurplePixel) {
            PurplePixel.LOWER_Y = yLower;
            PurplePixel.UPPER_Y = yUpper;
            PurplePixel.LOWER_CR = crLower;
            PurplePixel.UPPER_CR = crUpper;
            PurplePixel.LOWER_CB = cbLower;
            PurplePixel.UPPER_CB = cbUpper;
        } else if (processor instanceof YellowPixel) {
            YellowPixel.LOWER_Y = yLower;
            YellowPixel.UPPER_Y = yUpper;
            YellowPixel.LOWER_CR = crLower;
            YellowPixel.UPPER_CR = crUpper;
            YellowPixel.LOWER_CB = cbLower;
            YellowPixel.UPPER_CB = cbUpper;
        } else if (processor instanceof GreenPixel) {
            GreenPixel.LOWER_Y = yLower;
            GreenPixel.UPPER_Y = yUpper;
            GreenPixel.LOWER_CR = crLower;
            GreenPixel.UPPER_CR = crUpper;
            GreenPixel.LOWER_CB = cbLower;
            GreenPixel.UPPER_CB = cbUpper;
        }
    }
}
