package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data

import android.util.Size
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import java.util.Optional

/**
 * Data class for storing rectangular OpenCV contour information.
 *
 * @since 1.0.0-pre
 */
data class ContourData(
    /**
     * The rectangular bounding box representing this contour.
     */
    val rect: RotatedRect,
    /**
     * The points that make up this contour.
     */
    val points: List<Point>,
    /**
     * The area of the contour bounding box.
     */
    val area: Double,
    /**
     * The percentage of the screen the contour bounding box takes up.
     */
    val areaPercent: Double,
    /**
     * The aspect ratio of the contour bounding box.
     */
    val aspectRatio: Double,
    /**
     * The x coordinate of the center of the contour bounding box.
     */
    val centerX: Double,
    /**
     * The y coordinate of the center of the contour bounding box.
     */
    val centerY: Double,
    /**
     * The measured yaw of the contour bounding box.
     */
    val yaw: Double,
    /**
     * The measured pitch of the contour bounding box.
     */
    val pitch: Double,
    /**
     * The measured angle of the contour bounding box.
     */
    val angle: Measure<Angle>,
    /**
     * Optional PnP data (ordered as Translation Vector, Rotation Vector).
     */
    var pnp: Optional<Pair<Mat, Mat>> = Optional.empty()
) : VisionData() {
    @JvmOverloads
    constructor(cameraResolution: Size, points: Array<Point>, rect: RotatedRect, tvec: Mat? = null, rvec: Mat? = null) : this(
        rect,
        points.toList(),
        rect.boundingRect().area(),
        rect.boundingRect().area() / (cameraResolution.width * cameraResolution.height) * 100.0,
        rect.boundingRect().width.toDouble() / rect.boundingRect().height.toDouble(),
        rect.boundingRect().x + rect.boundingRect().width / 2.0,
        rect.boundingRect().y + rect.boundingRect().height / 2.0,
        (((rect.boundingRect().x + rect.boundingRect().width / 2.0) - cameraResolution.width / 2.0) / (cameraResolution.width / 2.0)),
        -(((rect.boundingRect().y + rect.boundingRect().height / 2.0) - cameraResolution.height / 2.0) / (cameraResolution.height / 2.0)),
        Degrees.of(ang(rect.angle, rect.size.width, rect.size.height)),
        if (tvec == null || rvec == null) Optional.empty() else Optional.of(tvec to rvec)
    )

    companion object {
        /**
         * Get the largest contour from a list of contours.
         */
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }

        private fun ang(angDeg: Double, width: Double, height: Double): Double {
            var ang = angDeg
            if (width < height)
                ang += 90.0
            return -ang + 180.0
        }
    }
}
