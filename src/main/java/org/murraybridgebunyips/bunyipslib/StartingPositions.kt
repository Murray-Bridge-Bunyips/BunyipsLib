package org.murraybridgebunyips.bunyipslib

/**
 * Enum for determining where the robot is starting on the field. This can be used to determine
 * which autonomous path to take.
 */
enum class StartingPositions {
    /**
     * FTC Field starting position Red Alliance, Left side if viewed from the Red Alliance.
     */
    STARTING_RED_LEFT,

    /**
     * FTC Field starting position Red Alliance, Right side if viewed from the Red Alliance.
     */
    STARTING_RED_RIGHT,

    /**
     * FTC Field starting position Blue Alliance, Left side if viewed from the Blue Alliance.
     */
    STARTING_BLUE_LEFT,

    /**
     * FTC Field starting position Blue Alliance, Right side if viewed from the Blue Alliance.
     */
    STARTING_BLUE_RIGHT;

    companion object {
        /**
         * Convert StartingPositions into a list of OpModeSelections. Useful in ABOM setOpModes().
         *
         * These positions are arranged so they will appear on the controller in the same order
         * as if the controller were rotated 45 degrees anti-clockwise and ABXY represented the four
         * positions from the audience's perspective.
         */
        @JvmStatic
        fun use(): List<OpModeSelection> {
            return listOf(
                OpModeSelection(STARTING_RED_LEFT), // A
                OpModeSelection(STARTING_RED_RIGHT), // B
                OpModeSelection(STARTING_BLUE_RIGHT), // X
                OpModeSelection(STARTING_BLUE_LEFT) // Y
            )
        }
    }
}
