package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data

import android.util.Size
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
    private var cameraDimensions: Size? = null

    /**
     * Alternate constructor that supports the usage of the [toContourData] method, which needs camera dimensions.
     */
    constructor(
        cameraDimensions: Size?,
        contour: MatOfPoint,
        contourPoints: List<Point>,
        contourArea: Int,
        density: Double,
        aspectRatio: Double,
        boxFit: RotatedRect
    ) : this(contour, contourPoints, contourArea, density, aspectRatio, boxFit) {
        this.cameraDimensions = cameraDimensions
    }

    /**
     * Convert the [ColourBlob] information stored by this instance into [ContourData] information.
     *
     * This method is designed for already implemented tasks such as the align/move to contour tasks, which take in
     * instances of [ContourData]. Therefore, it is possible to convert an entire list of [ColourBlob] readings
     * into a [ContourData] list with a simple streaming operation:
     * ```
     * () -> proc.getData().stream().map(ColourBlob::toContourData).collect(Collectors.toList())
     * ```
     * which is optionally allowed to be passed into these tasks as a Supplier.
     *
     * @since 5.1.0
     */
    fun toContourData(): ContourData {
        if (cameraDimensions == null)
            throw IllegalStateException("Camera dimensions for this ColourBlob data instance were not provided on construction, meaning it is not possible to convert this to a ContourData.")
        return ContourData(cameraDimensions!!, boxFit)
    }

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
