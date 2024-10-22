package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data

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
    val rgb: Int,
    /**
     * The exact Red RGB component of the dominant colour within the Region of Interest.
     */
    val red: Int,
    /**
     * The exact Green RGB component of the dominant colour within the Region of Interest.
     */
    val green: Int,
    /**
     * The exact Blue RGB component of the dominant colour within the Region of Interest.
     */
    val blue: Int
) : VisionData() {
    constructor(closestSwatch: Swatch, rgb: Int) : this(
        closestSwatch, rgb, (rgb shr 16) and 0xFF, (rgb shr 8) and 0xFF, rgb and 0xFF
    )
}