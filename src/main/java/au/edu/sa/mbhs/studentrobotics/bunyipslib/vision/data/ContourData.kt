package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data

import android.util.Size
import org.opencv.core.Rect

/**
 * Data class for storing contour data from a ColourThreshold.
 * @since 1.0.0-pre
 */
data class ContourData(
    /**
     * The bounding rectangle of the contour.
     */
    val boundingRect: Rect,
    /**
     * The area of the contour.
     */
    val area: Double,
    /**
     * The percentage of the screen the contour takes up.
     */
    val areaPercent: Double,
    /**
     * The aspect ratio of the contour.
     */
    val aspectRatio: Double,
    /**
     * The x coordinate of the center of the contour.
     */
    val centerX: Double,
    /**
     * The y coordinate of the center of the contour.
     */
    val centerY: Double,
    /**
     * The yaw of the contour in degrees.
     */
    val yaw: Double,
    /**
     * The pitch of the contour in degrees.
     */
    val pitch: Double
) : VisionData() {
    constructor(cameraResolution: Size, boundingRect: Rect) : this(
        boundingRect,
        boundingRect.area(),
        boundingRect.area() / (cameraResolution.width * cameraResolution.height) * 100.0,
        boundingRect.width.toDouble() / boundingRect.height.toDouble(),
        boundingRect.x + boundingRect.width / 2.0,
        boundingRect.y + boundingRect.height / 2.0,
        (((boundingRect.x + boundingRect.width / 2.0) - cameraResolution.width / 2.0) / (cameraResolution.width / 2.0)),
        -(((boundingRect.y + boundingRect.height / 2.0) - cameraResolution.height / 2.0) / (cameraResolution.height / 2.0))
    )

    companion object {
        /**
         * Get the largest contour from a list of contours.
         */
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }
    }
}
