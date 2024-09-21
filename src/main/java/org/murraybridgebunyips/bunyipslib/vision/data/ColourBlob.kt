package org.murraybridgebunyips.bunyipslib.vision.data

import com.qualcomm.robotcore.util.SortOrder
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.RotatedRect

/**
 * Data class for storing data related to the return results of a ColorBlobLocatorProcessor.
 * @since 5.0.0
 */
data class ColourBlob(
    /**
     * The OpenCV contour corresponding to this blob.
     */
    val contour: MatOfPoint,
    /**
     * The contour points for this blob.
     */
    val contourPoints: List<Point>,
    /**
     * The enclosed area by this blob's contour.
     */
    val contourArea: Int,
    /**
     * The density i.e. ratio of the contour area to convex hull area.
     */
    val density: Double,
    /**
     * The aspect ratio i.e. the ratio of the longer side of the bounding
     * box to the shorter side.
     */
    val aspectRatio: Double,
    /**
     * "Best fit" bounding box for this blob.
     */
    val boxFit: RotatedRect
) : VisionData() {
    /**
     * Collection of utility functions found via the Util inner class of ColorBlobLocatorProcessor.
     */
    companion object {
        /**
         * Filter blobs by minimum and maximum area.
         */
        @JvmStatic
        fun filterByArea(minArea: Int, maxArea: Int, blobs: List<ColourBlob>): List<ColourBlob> {
            return blobs.filter { it.contourArea in minArea..maxArea }
        }

        /**
         * Sorts a list of blobs by area.
         */
        @JvmStatic
        fun sortByArea(sortOrder: SortOrder, blobs: List<ColourBlob>): List<ColourBlob> {
            return blobs.sortedWith(compareBy<ColourBlob> { it.contourArea }.let {
                if (sortOrder == SortOrder.ASCENDING) it else it.reversed()
            })
        }

        /**
         * Filter blobs by minimum and maximum density.
         */
        @JvmStatic
        fun filterByDensity(minDensity: Double, maxDensity: Double, blobs: List<ColourBlob>): List<ColourBlob> {
            return blobs.filter { it.density in minDensity..maxDensity }
        }

        /**
         * Sorts a list of blobs by density.
         */
        @JvmStatic
        fun sortByDensity(sortOrder: SortOrder, blobs: List<ColourBlob>): List<ColourBlob> {
            return blobs.sortedWith(compareBy<ColourBlob> { it.density }.let {
                if (sortOrder == SortOrder.ASCENDING) it else it.reversed()
            })
        }

        /**
         * Filter blobs by minimum and maximum aspect ratio.
         */
        @JvmStatic
        fun filterByAspectRatio(
            minAspectRatio: Double,
            maxAspectRatio: Double,
            blobs: List<ColourBlob>
        ): List<ColourBlob> {
            return blobs.filter { it.aspectRatio in minAspectRatio..maxAspectRatio }
        }

        /**
         * Sorts a list of blobs by aspect ratio.
         */
        @JvmStatic
        fun sortByAspectRatio(sortOrder: SortOrder, blobs: List<ColourBlob>): List<ColourBlob> {
            return blobs.sortedWith(compareBy<ColourBlob> { it.aspectRatio }.let {
                if (sortOrder == SortOrder.ASCENDING) it else it.reversed()
            })
        }
    }
}
