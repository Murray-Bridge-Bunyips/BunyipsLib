package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data

import android.util.Size
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.vision.opencv.Circle
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobCriteria
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import kotlin.math.sign

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
    val boxFit: RotatedRect,
    /**
     * The arc length for this blob.
     */
    val arcLength: Double,
    /**
     * The circularity for this blob.
     */
    val circularity: Double,
    /**
     * The center Point and radius of the circle enclosing this blob.
     */
    val circle: Circle
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
        boxFit: RotatedRect,
        arcLength: Double,
        circularity: Double,
        circle: Circle
    ) : this(contour, contourPoints, contourArea, density, aspectRatio, boxFit, arcLength, circularity, circle) {
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
        return ContourData(cameraDimensions!!, contourPoints.toTypedArray(), boxFit)
    }

    /**
     * Collection of in-place sorting and filtering functions found via the Util inner class of ColorBlobLocatorProcessor.
     */
    companion object {
        /**
         * Remove from a mutable list of [ColourBlob]s those which fail to meet a given criteria.
         *
         * @param criteria criteria by which to filter by
         * @param minValue minimum value
         * @param maxValue maximum value
         * @param blobs List of [ColourBlob]s to operate on
         */
        fun filterByCriteria(
            criteria: BlobCriteria,
            minValue: Double,
            maxValue: Double,
            blobs: MutableList<ColourBlob>
        ) {
            val toRemove = ArrayList<ColourBlob>()
            for (b in blobs) {
                val value = when (criteria) {
                    BlobCriteria.BY_CONTOUR_AREA -> b.contourArea.toDouble()
                    BlobCriteria.BY_DENSITY -> b.density
                    BlobCriteria.BY_ASPECT_RATIO -> b.aspectRatio
                    BlobCriteria.BY_ARC_LENGTH -> b.arcLength
                    BlobCriteria.BY_CIRCULARITY -> b.circularity
                }
                if (value !in minValue..maxValue)
                    toRemove.add(b)
            }
            blobs.removeAll(toRemove)
        }

        /**
         * Rearrange a mutable list of [ColourBlob]s to sort by a criteria.
         *
         * @param criteria criteria by which to sort by
         * @param sortOrder sort order, ascending or descending
         * @param blobs List of [ColourBlob]s to operate on
         */
        fun sortByCriteria(criteria: BlobCriteria, sortOrder: SortOrder, blobs: MutableList<ColourBlob>) {
            blobs.sortWith { c1, c2 ->
                var diff = when (criteria) {
                    BlobCriteria.BY_CONTOUR_AREA -> sign(c2.contourArea.toDouble() - c1.contourArea)
                    BlobCriteria.BY_DENSITY -> sign(c2.density - c1.density)
                    BlobCriteria.BY_ASPECT_RATIO -> sign(c2.aspectRatio - c1.aspectRatio)
                    BlobCriteria.BY_ARC_LENGTH -> sign(c2.arcLength - c1.arcLength)
                    BlobCriteria.BY_CIRCULARITY -> sign(c2.circularity - c1.circularity)
                }
                if (sortOrder == SortOrder.ASCENDING)
                    diff = -diff
                diff.toInt()
            }
        }
    }
}
