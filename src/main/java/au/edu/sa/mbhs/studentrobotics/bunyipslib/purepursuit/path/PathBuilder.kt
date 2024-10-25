package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapDeltaRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf.wrapRadians
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.ConstantInterpolator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.LinearInterpolator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.SplineInterpolator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.interpolator.TangentInterpolator
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry.approx
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry.approxNormHeading
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry.distTo
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * [Source](https://github.com/acmerobotics/road-runner/tree/v0.5.6/core/src/main/kotlin/com/acmerobotics/roadrunner/path)
 *
 * @param startPose start pose
 * @param startTangent start tangent
 * @param path previous path
 * @param s displacement in previous path
 * @since 6.0.0
 */
class PathBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    internal val path: Path?,
    internal val s: Double?
) {
    @JvmOverloads
    constructor(startPose: Pose2d, startTangent: Double = startPose.heading.toDouble()) :
            this(startPose, startTangent, null, null)

    constructor(startPose: Pose2d, reversed: Boolean) :
            this(startPose, (startPose.heading.toDouble() + if (reversed) PI else 0.0).wrapRadians())

    constructor(path: Path, s: Double) : this(null, null, path, s)

    private var currentPose: Pose2d? = startPose
    private var currentTangent: Double? = startTangent

    private var segments = mutableListOf<PathSegment>()

    private fun makeLine(end: Vector2d): LineSegment {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (start.position approx end) {
            throw RuntimeException("Empty path!")
        }

        return LineSegment(start.position, end)
    }

    private fun makeSpline(endPosition: Vector2d, endTangent: Double): QuinticSpline {
        val startPose = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (startPose.position approx endPosition) {
            throw RuntimeException("Empty path!")
        }

        val derivMag = startPose.position distTo endPosition
        val (startWaypoint, endWaypoint) = if (currentPose == null) {
            val startDeriv = path!!.internalDeriv(s!!).position
            val startSecondDeriv = path.internalSecondDeriv(s).position
            QuinticSpline.Knot(startPose.position, startDeriv, startSecondDeriv) to
                    QuinticSpline.Knot(endPosition, Vector2d(derivMag * cos(endTangent), derivMag * sin(endTangent)))
        } else {
            QuinticSpline.Knot(startPose.position, Vector2d(derivMag, currentTangent!!)) to
                    QuinticSpline.Knot(endPosition, Vector2d(derivMag * cos(endTangent), derivMag * sin(endTangent)))
        }

        return QuinticSpline(startWaypoint, endWaypoint)
    }

    private fun makeTangentInterpolator(curve: ParametricCurve): TangentInterpolator {
        if (currentPose == null) {
            val prevInterpolator = path!!.segment(s!!).first.interpolator
            if (prevInterpolator !is TangentInterpolator) {
                throw RuntimeException("Continuity violation!")
            }
            return TangentInterpolator(prevInterpolator.offset)
        }

        val startHeading = curve.tangentAngle(0.0, 0.0)

        val interpolator = TangentInterpolator(currentPose!!.heading.toDouble() - startHeading)
        interpolator.init(curve)
        return interpolator
    }

    private fun makeConstantInterpolator(): ConstantInterpolator {
        val currentHeading = currentPose?.heading ?: throw RuntimeException("Continuity violation!")

        return ConstantInterpolator(currentHeading.toDouble())
    }

    private fun makeLinearInterpolator(endHeading: Double): LinearInterpolator {
        val startHeading = currentPose?.heading ?: throw RuntimeException("Continuity violation!")

        return LinearInterpolator(startHeading.toDouble(), (endHeading - startHeading.toDouble()).wrapDeltaRadians())
    }

    private fun makeSplineInterpolator(endHeading: Double): SplineInterpolator {
        return if (currentPose == null) {
            SplineInterpolator(
                path!![s!!].heading.toDouble(),
                endHeading,
                path.deriv(s).heading.toDouble(),
                path.secondDeriv(s).heading.toDouble(),
                null,
                null
            )
        } else {
            SplineInterpolator(currentPose?.heading!!.toDouble(), endHeading)
        }
    }

    private fun addSegment(segment: PathSegment): PathBuilder {
        if (segments.isNotEmpty()) {
            val lastSegment = segments.last()
            if (!(lastSegment.end() approxNormHeading segment.start() &&
                        lastSegment.endDeriv() approx segment.startDeriv() &&
                        lastSegment.endSecondDeriv().position approx segment.startSecondDeriv().position)
            ) {
                throw RuntimeException("Continuity violation!")
            }
        } else if (currentPose == null) {
            if (!(path!![s!!] approxNormHeading segment.start() &&
                        path.deriv(s) approx segment.startDeriv() &&
                        path.secondDeriv(s).position approx segment.startSecondDeriv().position)
            ) {
                throw RuntimeException("Continuity violation!")
            }
        }

        currentPose = segment.end()
        currentTangent = segment.endTangentAngle()

        segments.add(segment)

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d): PathBuilder {
        val line = makeLine(endPosition)
        val interpolator = makeTangentInterpolator(line)

        addSegment(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d) =
        addSegment(PathSegment(makeLine(endPosition), makeConstantInterpolator()))

    /**
     * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d) = lineToConstantHeading(endPosition)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToLinearHeading(endPose: Pose2d) =
        addSegment(PathSegment(makeLine(endPose.position), makeLinearInterpolator(endPose.heading.toDouble())))

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToSplineHeading(endPose: Pose2d) =
        addSegment(PathSegment(makeLine(endPose.position), makeSplineInterpolator(endPose.heading.toDouble())))

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return lineTo(
            start.position + Vector2d(
                distance * cos(start.heading.toDouble()),
                distance * sin(start.heading.toDouble())
            )
        )
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): PathBuilder {
        forward(-distance)
        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return strafeTo(
            start.position + Vector2d(
                distance * cos(start.heading.toDouble() + PI / 2),
                distance * sin(start.heading.toDouble() + PI / 2)
            )
        )
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): PathBuilder {
        return strafeLeft(-distance)
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineTo(endPosition: Vector2d, endTangent: Double): PathBuilder {
        val spline = makeSpline(endPosition, endTangent)
        val interpolator = makeTangentInterpolator(spline)

        return addSegment(PathSegment(spline, interpolator))
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double) =
        addSegment(PathSegment(makeSpline(endPosition, endTangent), makeConstantInterpolator()))

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToLinearHeading(endPose: Pose2d, endTangent: Double) =
        addSegment(
            PathSegment(
                makeSpline(endPose.position, endTangent),
                makeLinearInterpolator(endPose.heading.toDouble())
            )
        )

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToSplineHeading(endPose: Pose2d, endTangent: Double) =
        addSegment(
            PathSegment(
                makeSpline(endPose.position, endTangent),
                makeSplineInterpolator(endPose.heading.toDouble())
            )
        )

    /**
     * Constructs the [Path] instance.
     */
    fun build(): Path {
        return Path(segments)
    }
}