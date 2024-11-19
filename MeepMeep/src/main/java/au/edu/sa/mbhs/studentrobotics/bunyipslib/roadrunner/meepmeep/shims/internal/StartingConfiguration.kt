package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.StartingConfiguration.Alliance.BLUE
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.StartingConfiguration.Alliance.RED
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.StartingConfiguration.Origin.LEFT
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.StartingConfiguration.Origin.RIGHT
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Degrees
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Feet
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.FieldTiles
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Inches
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.Radians
import com.acmerobotics.roadrunner.Pose2d
import kotlin.math.abs

/**
 * Shim for StartingConfiguration to allow for easy initial pose construction.
 *
 * In games such as 2024-2025 INTO THE DEEP, there are no longer strict starting positions, and instead can be
 * along any legal zone on the alliance wall. This class allows an alliance to be selected, then a Field Tile
 * dimension to translate across the back wall. This process is exposed as a static builder and allows unlimited
 * customisation of pose data while maintaining readability.
 *
 * @author Lucas Bubner, 2024
 */
object StartingConfiguration {
    /**
     * Represents a position that a robot is able to start a match in.
     */
    data class Position @JvmOverloads constructor(
        /**
         * The alliance this robot is starting in.
         */
        val alliance: Alliance,
        /**
         * The side the robot is biased or has been placed on.
         */
        val origin: Origin,
        /**
         * The translation from the center of the field tile to the robot center, in order to touch the field wall.
         */
        val backwardTranslation: Measure<Distance>,
        /**
         * The translation from the origin to the robot center.
         */
        val horizontalTranslation: Measure<Distance>,
        /**
         * Counter-clockwise rotation of the robot in the starting position.
         */
        val ccwRotation: Measure<Angle>,
        /**
         * Read-only arbitrary user flags for this starting configuration.
         */
        val flags: Set<Any> = emptySet()
    ) {
        /**
         * Convert this starting configuration into the FTC Field Coordinate/RoadRunner coordinate system.
         */
        fun toFieldPose(): Pose2d {
            // Zero to red alliance, left side of the field
            val xZeroInch = -72.0 * origin.directionMultiplier * alliance.directionMultiplier
            val yZeroInch = -60.0 * alliance.directionMultiplier
            return Pose2d(
                xZeroInch + (horizontalTranslation to Inches) * origin.directionMultiplier * alliance.directionMultiplier,
                yZeroInch - (backwardTranslation to Inches) * alliance.directionMultiplier,
                alliance.directionMultiplier * Math.PI / 2.0 + (ccwRotation to Radians)
            )
        }

        /**
         * Whether this starting configuration is on the red alliance.
         */
        val isRed by lazy { alliance == RED }

        /**
         * Whether this starting configuration is on the blue alliance.
         */
        val isBlue by lazy { alliance == BLUE }

        /**
         * Whether this starting configuration is left-biased.
         */
        val isLeft by lazy { origin == LEFT }

        /**
         * Whether this starting configuration is right-biased.
         */
        val isRight by lazy { origin == RIGHT }

        /**
         * Invert this starting configuration over the center of the field, returning a new starting configuration
         * that is a direct mirror on the other alliance (symmetrical mirror).
         */
        fun invert(): Position {
            return Position(alliance.invert(), origin.invert(), backwardTranslation, horizontalTranslation, ccwRotation)
        }

        /**
         * Return an informative string about this starting configuration.
         */
        fun toVerboseString(): String {
            return "{alliance=${alliance.name}, origin=${origin.name}, backwardTranslation=${backwardTranslation}, horizontalTranslation=${horizontalTranslation}, ccwRotation=${ccwRotation}, fieldPose=${toFieldPose()}}"

        }
    }

    /**
     * Represents the bias or placement of the robot parallel to the field wall.
     */
    enum class Origin(
        /**
         * Multiplicative quantifier, [LEFT] is normal and [RIGHT] is inverted.
         */
        val directionMultiplier: Int
    ) {
        /**
         * Left-bias or placement.
         */
        LEFT(1),

        /**
         * Right-bias or placement.
         */
        RIGHT(-1);

        /**
         * Invert this origin to the opposite side.
         */
        fun invert(): Origin {
            return if (this == LEFT) RIGHT else LEFT
        }
    }

    /**
     * Represents the player alliance of the robot.
     */
    enum class Alliance(
        /**
         * Multiplicative quantifier, [RED] is normal and [BLUE] is inverted.
         */
        val directionMultiplier: Int
    ) {
        /**
         * The Red Alliance
         */
        RED(1),

        /**
         * The Blue Alliance
         */
        BLUE(-1);

        /**
         * Invert this alliance to be the opposite alliance.
         */
        fun invert(): Alliance {
            return if (this == RED) BLUE else RED
        }
    }

    /**
     * Create a new [StartingConfiguration.Position] with an origin on the `BLUE` alliance with a `LEFT` origin.
     */
    @JvmStatic
    fun blueLeft(): Builder {
        return Builder(BLUE, LEFT)
    }

    /**
     * Create a new [StartingConfiguration.Position] with an origin on the `BLUE` alliance with a `RIGHT` origin.
     */
    @JvmStatic
    fun blueRight(): Builder {
        return Builder(BLUE, RIGHT)
    }

    /**
     * Create a new [StartingConfiguration.Position] with an origin on the `RED` alliance with a `LEFT` origin.
     */
    @JvmStatic
    fun redLeft(): Builder {
        return Builder(RED, LEFT)
    }

    /**
     * Create a new [StartingConfiguration.Position] with an origin on the `RED` alliance with a `RIGHT` origin.
     */
    @JvmStatic
    fun redRight(): Builder {
        return Builder(RED, RIGHT)
    }

    /**
     * Utility builder for a [StartingConfiguration.Position].
     */
    class Builder(private val alliance: Alliance, private val origin: Origin) {
        private lateinit var horizontalTranslation: Measure<Distance>
        private var back: Measure<Distance> = FieldTiles.zero()
        private var rotation: Measure<Angle> = Degrees.zero()

        /**
         * Sets the translational index of the field tile to start on (number 0.5-6.5, fractional values permitted for half-tiles).
         * This translation is positioned in the vertical center of the field tile, starting from the side of the origin.
         */
        fun tile(tileFromOrigin: Double): PrebuiltPosition {
            require(tileFromOrigin in 0.5..6.5) {
                "Tile from origin must be between 0.5 and 6.5, got $tileFromOrigin"
            }
            translate(FieldTiles.one() / 2.0 + FieldTiles.one() * (tileFromOrigin - 1.0))
            return PrebuiltPosition()
        }

        /**
         * Translate over from the origin (field wall) to choose the robot starting translation.
         * This translation is positioned in the vertical center of the field tile, starting from the side of the origin.
         */
        fun translate(translationFromOrigin: Measure<Distance>): PrebuiltPosition {
            val mag = abs(translationFromOrigin to Feet)
            require(mag < 12.0) {
                "Translation from origin must be less than 12 feet, got $mag feet"
            }
            horizontalTranslation = translationFromOrigin
            return PrebuiltPosition()
        }

        /**
         * A position that can hold extra attributes including rotation and forward translation, allowing flags
         * to be attached to the position.
         *
         * The build method is automatically called if forgotten via the AutonomousBunyipsOpMode `setOpModes()` method, ensure
         * other implementations are aware of this potential runtime error (attempting to parse a prebuilt position).
         */
        inner class PrebuiltPosition {
            private val flags = mutableSetOf<Any>()

            /**
             * Translate backward from the center of the field tile to your robot center as defined by your translate step.
             * This is to align your robot to touching the field wall.
             */
            fun backward(translationBack: Measure<Distance>): PrebuiltPosition {
                back = translationBack
                return this
            }

            /**
             * Rotate the starting position in-place to represent what angle your robot is facing *relative to facing
             * forward* on the chosen alliance side.
             */
            fun rotate(ccwRotation: Measure<Angle>): PrebuiltPosition {
                rotation = ccwRotation
                return this
            }

            /**
             * Add a flag to this starting configuration, which can be used to additionally
             * identify the position in a custom way.
             */
            fun flag(flag: Any): PrebuiltPosition {
                flags.add(flag)
                return this
            }

            /**
             * Construct this position.
             */
            fun build(): Position {
                return Position(alliance, origin, back, horizontalTranslation, rotation, flags.toSet())
            }
        }
    }
}
