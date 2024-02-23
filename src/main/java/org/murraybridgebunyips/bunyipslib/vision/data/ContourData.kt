package org.murraybridgebunyips.bunyipslib.vision.data

import org.murraybridgebunyips.bunyipslib.vision.Vision
import org.opencv.core.Rect

data class ContourData(
    val boundingRect: Rect,
) : VisionData() {
    val area: Double = boundingRect.area()
    val areaPercent: Double = boundingRect.area() / (Vision.CAMERA_WIDTH * Vision.CAMERA_HEIGHT) / 100
    val aspectRatio: Double = boundingRect.width.toDouble() / boundingRect.height.toDouble()
    val centerX: Double = boundingRect.x + boundingRect.width / 2.0
    val centerY: Double = boundingRect.y + boundingRect.height / 2.0
    val yaw: Double = (centerX - Vision.CAMERA_WIDTH / 2.0) / Vision.CAMERA_WIDTH
    // -2.66 arbitrary modifier to scale from -1 to 1, bit sketchy; will scale so 1 is furthest away, -1
    // is at the bottom of the camera, 0 is in the middle. Probably a good idea to make this more accurate.
    val pitch: Double = (centerY - Vision.CAMERA_HEIGHT / 2.0) / Vision.CAMERA_HEIGHT * -2.66

    companion object {
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }
    }
}
