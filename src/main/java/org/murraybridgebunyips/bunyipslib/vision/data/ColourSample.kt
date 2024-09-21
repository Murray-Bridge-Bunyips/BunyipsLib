package org.murraybridgebunyips.bunyipslib.vision.data

import androidx.annotation.IntRange
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch

/**
 * Data class for storing data related to the return results of a PredominantColorProcessor.
 * This data class will only have one instance affixed to the corresponding ColourSensor.
 * @since 5.0.0
 */
data class ColourSample(
    /**
     * The "best guess" at the general shade of the dominant colour in the Region of Interest.
     */
    val closestSwatch: Swatch,
    /**
     * The exact numerical value of the dominant colour in the Region of Interest.
     */
    val rgb: Int
) : VisionData() {
    companion object {
        /**
         * Utility to convert an RGB integer to a 3-wide array of R, G, and B components.
         * See the Android Color class for more conversion details.
         */
        @JvmStatic
        @IntRange(from = 0, to = 255)
        fun toRGBComponents(rgb: Int): Array<Int> {
            return arrayOf(
                (rgb shr 16) and 0xFF,
                (rgb shr 8) and 0xFF,
                rgb and 0xFF
            )
        }
    }
}