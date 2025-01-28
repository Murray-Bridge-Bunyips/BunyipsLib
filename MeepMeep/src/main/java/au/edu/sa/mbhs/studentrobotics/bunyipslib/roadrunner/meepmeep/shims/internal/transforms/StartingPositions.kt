package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.transforms

import com.acmerobotics.roadrunner.Pose2d

/**
 * Legacy enum for determining where the robot is starting on the field. This can be used to determine
 * which autonomous path to take.
 *
 * **Note**: This enum was designed around the 4 *required* starting positions as required by an Autonomous Randomisation
 * task. In a season such as 2024-2025 INTO THE DEEP where these required positions are no longer required, the pose
 * information as granted by these enum values may be inaccurate (as they are derived from CENTERSTAGE).
 * However, this enum may still be used for differentiating between the relative position of robots in their alliance, but comes
 * with no guarantee that the pose data will be valid. The pose property has been left mutable for advanced uses of this class in specific.
 *
 * See the more flexible [StartingConfiguration] class for handling robot start pose against any position on the alliance wall.
 *
 * @author Lucas Bubner, 2024
 * @see StartingConfiguration
 * @since 1.0.0-pre
 */
enum class StartingPositions(
    /**
     * The pose of the starting position. This is the position of the robot on the field when the
     * match starts in the FTC Field Coordinate system (+ RoadRunner). The heading value is if the robot were to
     * face inwards towards the field.
     *
     * **Note!** These default tile poses and vector relates to the starting positions
     * in CENTERSTAGE, see the more flexible [StartingConfiguration] class for handling robot start pose.
     *
     * This pose has been left mutable to allow for advanced use cases.
     * Units: Inches, Radians
     */
    var pose: Pose2d
) {
    /**
     * Represents a robot starting on the left side of the red alliance.
     *
     * Default pose data is set to FTC Field Tile `F2` on the Red Alliance, robot on left side of field from red's POV.
     * Pose heading faces inwards towards the field.
     */
    STARTING_RED_LEFT(Pose2d(-36.0, -60.0, Math.PI / 2.0)),

    /**
     * Represents a robot starting on the right side of the red alliance.
     *
     * Default pose data is set to FTC Field Tile `F4` on the Red Alliance, robot on right side of field from red's POV.
     * Pose heading faces inwards towards the field.
     */
    STARTING_RED_RIGHT(Pose2d(12.0, -60.0, Math.PI / 2.0)),

    /**
     * Represents a robot starting on the left side of the blue alliance.
     *
     * Default pose data is set to FTC Field Tile `A4` on the Blue Alliance, robot on left side of field from blue's POV.
     * Pose heading faces inwards towards the field.
     */
    STARTING_BLUE_LEFT(Pose2d(12.0, 60.0, -Math.PI / 2.0)),

    /**
     * Represents a robot starting on the right side of the blue alliance.
     *
     * Default pose data is set to FTC Field Tile `A2` on the Blue Alliance, robot on right side of field from blue's POV.
     * Pose heading faces inwards towards the field.
     */
    STARTING_BLUE_RIGHT(Pose2d(-36.0, 60.0, -Math.PI / 2.0));

    /**
     * The vector of the starting position with no heading information.
     *
     * **Note!** These default tile poses and vector relates to the starting positions
     * in CENTERSTAGE, see the more flexible [StartingConfiguration] class for handling robot start pose.
     *
     * Units: Inches
     */
    val vector by lazy { pose.position }

    /**
     * Whether the starting position is on the red alliance side of the field.
     */
    val isRed by lazy { this == STARTING_RED_LEFT || this == STARTING_RED_RIGHT }

    /**
     * Whether the starting position is on the blue alliance side of the field.
     */
    val isBlue by lazy { this == STARTING_BLUE_LEFT || this == STARTING_BLUE_RIGHT }

    /**
     * Whether the starting position is on the left side of the field.
     */
    val isLeft by lazy { this == STARTING_RED_LEFT || this == STARTING_BLUE_LEFT }

    /**
     * Whether the starting position is on the right side of the field.
     */
    val isRight by lazy { this == STARTING_RED_RIGHT || this == STARTING_BLUE_RIGHT }

    /**
     * Convert this starting position to a starting configuration.
     */
    fun toStartingConfiguration(): StartingConfiguration.Position {
        return when (this) {
            STARTING_RED_LEFT -> StartingConfiguration.redLeft().tile(2.0).build()
            STARTING_RED_RIGHT -> StartingConfiguration.redRight().tile(3.0).build()
            STARTING_BLUE_LEFT -> StartingConfiguration.blueLeft().tile(3.0).build()
            STARTING_BLUE_RIGHT -> StartingConfiguration.blueRight().tile(2.0).build()
        }
    }

    override fun toString(): String {
        return getHTMLIfAvailable(this)
    }

    companion object {
        /**
         * Convert StartingPositions into an array. Useful in ABOM setOpModes().
         *
         * These positions are arranged so they will appear on the controller in the same order
         * as if the controller were rotated 45 degrees anti-clockwise and ABXY represented the four
         * positions from the audience's perspective.
         */
        @JvmStatic /* giulio was here fun use(): list<any> { */
        fun use(): Array<Any> {
            return arrayOf(
                STARTING_RED_LEFT, // A
                STARTING_RED_RIGHT, // B /*giulio is is still here*/
                STARTING_BLUE_RIGHT, // X
                STARTING_BLUE_LEFT // Y
            )
        }

        /**
         * Get the HTML representation of the starting position if available.
         */
        fun getHTMLIfAvailable(startingPosition: Any?): String {
            return when (use().find { it == startingPosition }) {
                STARTING_RED_LEFT -> "<font color='red'>Red</font> Alliance, <i>Left</i>"
                STARTING_RED_RIGHT -> "<font color='red'>Red</font> Alliance, Right"
                STARTING_BLUE_LEFT -> "<font color='#3863ff'>Blue</font> Alliance, <i>Left</i>"
                STARTING_BLUE_RIGHT -> "<font color='#3863ff'>Blue</font> Alliance, Right"
                else -> startingPosition.toString()
            }
        }
    }
}
