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
     * The exact Red RGB component of the dominant colour within the Region of Interest. (0-255)
     */
    val red: Int,
    /**
     * The exact Green RGB component of the dominant colour within the Region of Interest. (0-255)
     */
    val green: Int,
    /**
     * The exact Blue RGB component of the dominant colour within the Region of Interest. (0-255)
     */
    val blue: Int,
    /**
     * The exact Hue HSV component of the dominant colour within the Region of Interest. (0-180)
     */
    val hue: Int,
    /**
     * The exact Saturation HSV component of the dominant colour within the Region of Interest. (0-255)
     */
    val saturation: Int,
    /**
     * The exact Value HSV component of the dominant colour within the Region of Interest. (0-255)
     */
    val value: Int,
    /**
     * The exact Luminance (Y) YCrCb component of the dominant colour within the Region of Interest. (0-255)
     */
    val luminance: Int,
    /**
     * The exact Red-difference Chroma (Cr) YCrCb component of the dominant colour within the Region of Interest. (0-255, center 128)
     */
    val chrominanceRed: Int,
    /**
     * The exact Blue-difference Chroma (Cb) YCrCb component of the dominant colour within the Region of Interest. (0-255, center 128)
     */
    val chrominanceBlue: Int
) : VisionData() {
    /**
     * requirement: [rgb], [hsv], [yCrCb] arrays must be three-wide containing equivalent colour components
     */
    constructor(closestSwatch: Swatch, rgb: IntArray, hsv: IntArray, yCrCb: IntArray) : this(
        closestSwatch, rgb[0], rgb[1], rgb[2], hsv[0], hsv[1], hsv[2], yCrCb[0], yCrCb[1], yCrCb[2]
    )
}