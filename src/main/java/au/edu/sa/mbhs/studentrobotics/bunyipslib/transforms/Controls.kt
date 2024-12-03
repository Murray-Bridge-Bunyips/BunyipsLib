package au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Cartesian
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad

/**
 * Utility class for the different button and analog controls on the gamepad.
 * Used for adding additional abstraction to the current gamepad control system, used across BunyipsLib.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
@Suppress("KDocMissingDocumentation")
enum class Controls {
    // The guide button not included due to volatility as it may be caught by the app
    A, B, X, Y, START, BACK, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON, NONE;

    /**
     * Represents the analog inputs on a gamepad.
     */
    enum class Analog {
        LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y, LEFT_TRIGGER, RIGHT_TRIGGER;

        companion object {
            /**
             * Get the value of an analog input on a gamepad.
             */
            @JvmStatic
            fun get(gamepad: Gamepad, analog: Analog): Float {
                return when (analog) {
                    LEFT_STICK_X -> gamepad.left_stick_x
                    LEFT_STICK_Y -> gamepad.left_stick_y
                    RIGHT_STICK_X -> gamepad.right_stick_x
                    RIGHT_STICK_Y -> gamepad.right_stick_y
                    LEFT_TRIGGER -> gamepad.left_trigger
                    RIGHT_TRIGGER -> gamepad.right_trigger
                }
            }

            /**
             * Get the value of an analog input on a gamepad, with the Y axis inverted for the left and right sticks.
             */
            @JvmStatic
            fun getCartesian(gamepad: Gamepad, analog: Analog): Float {
                return when (analog) {
                    LEFT_STICK_X -> gamepad.left_stick_x
                    LEFT_STICK_Y -> -gamepad.left_stick_y
                    RIGHT_STICK_X -> gamepad.right_stick_x
                    RIGHT_STICK_Y -> -gamepad.right_stick_y
                    LEFT_TRIGGER -> gamepad.left_trigger
                    RIGHT_TRIGGER -> gamepad.right_trigger
                }
            }
        }
    }

    /**
     * Groups of buttons that can be customised.
     */
    enum class ButtonGroup {
        /**
         * All bumpers
         */
        BUMPERS,

        /**
         * All face buttons
         */
        DPAD,

        /**
         * ABXY buttons
         */
        BUTTONS,

        /**
         * Start, back, and stick buttons
         */
        SPECIAL,

        /**
         * All buttons
         */
        ALL
    }


    /**
     * Groups of axes that can be customised.
     */
    enum class AnalogGroup {
        /**
         * Both analog sticks
         */
        STICKS,

        /**
         * Both triggers
         */
        TRIGGERS,

        /**
         * All analog inputs
         */
        ALL
    }


    companion object {
        /**
         * Check if a button is currently pressed on a gamepad.
         */
        @JvmStatic
        fun isSelected(gamepad: Gamepad, button: Controls): Boolean {
            var buttonPressed = false
            when (button) {
                DPAD_UP -> buttonPressed = gamepad.dpad_up
                DPAD_DOWN -> buttonPressed = gamepad.dpad_down
                DPAD_LEFT -> buttonPressed = gamepad.dpad_left
                DPAD_RIGHT -> buttonPressed = gamepad.dpad_right
                // Controller initialisation trigger safeguard
                A -> if (!gamepad.start) buttonPressed = gamepad.a
                B -> if (!gamepad.start) buttonPressed = gamepad.b
                X -> buttonPressed = gamepad.x
                Y -> buttonPressed = gamepad.y
                START -> buttonPressed = gamepad.start
                BACK -> buttonPressed = gamepad.back
                LEFT_BUMPER -> buttonPressed = gamepad.left_bumper
                RIGHT_BUMPER -> buttonPressed = gamepad.right_bumper
                LEFT_STICK_BUTTON -> buttonPressed = gamepad.left_stick_button
                RIGHT_STICK_BUTTON -> buttonPressed = gamepad.right_stick_button

                NONE -> {}
            }
            return buttonPressed
        }

        /**
         * Get the character representation of a button.
         */
        @JvmStatic
        fun getChar(button: Controls): Char {
            when (button) {
                DPAD_UP -> return 'u'
                DPAD_DOWN -> return 'd'
                DPAD_LEFT -> return 'l'
                DPAD_RIGHT -> return 'r'
                A -> return 'a'
                B -> return 'b'
                X -> return 'x'
                Y -> return 'y'
                START -> return '*'
                BACK -> return '-'
                LEFT_BUMPER -> return '\\'
                RIGHT_BUMPER -> return '/'
                LEFT_STICK_BUTTON -> return 'L'
                RIGHT_STICK_BUTTON -> return 'R'
                NONE -> return 'n'
            }
        }

        /**
         * Map an array of arguments to controller buttons in order of the enum.
         */
        @JvmStatic
        fun <T> mapArgs(args: Array<out T>): HashMap<T, Controls> {
            // Map strings of args to every controller enum in order
            if (args.size >= values().size) {
                throw IllegalArgumentException("Controller: Number of args exceeds number of possible gamepad buttons (14).")
            }
            val map = HashMap<T, Controls>()
            for (i in args.indices) {
                // For every arg, map it to the corresponding enum
                map[args[i]] = values()[i]
            }
            return map
        }

        /**
         * Convert the gamepad movement values into a robot velocity.
         * This inverts the y value as the gamepad y stick is inverted.
         */
        @JvmStatic
        fun vel(lsx: Double, lsy: Double, rsx: Double): PoseVelocity2d {
            return Cartesian.toVel(lsx, -lsy, rsx)
        }

        /**
         * Convert the gamepad translation values to a robot vector.
         * This inverts the y value as the gamepad y stick is inverted.
         */
        @JvmStatic
        fun vec(lsx: Double, lsy: Double): Vector2d {
            return Cartesian.toVec(lsx, -lsy)
        }

        /**
         * Return a string of all buttons and values currently pressed.
         */
        @JvmStatic
        fun movementString(gamepad: Gamepad): String {
            if (gamepad.id == -1) return "(dc)"
            var str = "("
            for (button in values()) {
                if (button == NONE) continue
                if (isSelected(gamepad, button)) {
                    str += getChar(button)
                }
            }
            if (gamepad.left_stick_y != 0.0f)
                str += "[ly]"
            if (gamepad.left_stick_x != 0.0f)
                str += "[lx]"
            if (gamepad.right_stick_y != 0.0f)
                str += "[ry]"
            if (gamepad.right_stick_x != 0.0f)
                str += "[rx]"
            if (gamepad.left_trigger != 0.0f)
                str += "[lt]"
            if (gamepad.right_trigger != 0.0f)
                str += "[rt]"
            if (str == "(")
                str += "n"
            str += ")"
            return str
        }

        /**
         * Get the buttons in a specified ButtonGroup.
         */
        @JvmStatic
        fun getButtons(group: ButtonGroup): Array<Controls> {
            return when (group) {
                ButtonGroup.BUMPERS -> arrayOf(LEFT_BUMPER, RIGHT_BUMPER)
                ButtonGroup.DPAD -> arrayOf(DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT)
                ButtonGroup.BUTTONS -> arrayOf(A, B, X, Y)
                ButtonGroup.SPECIAL -> arrayOf(START, BACK, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON)
                ButtonGroup.ALL -> values()
            }
        }

        /**
         * Get the axes in a specified AnalogGroup.
         */
        @JvmStatic
        fun getAxes(group: AnalogGroup): Array<Analog> {
            return when (group) {
                AnalogGroup.STICKS -> arrayOf(
                    Analog.LEFT_STICK_X,
                    Analog.LEFT_STICK_Y,
                    Analog.RIGHT_STICK_X,
                    Analog.RIGHT_STICK_Y
                )

                AnalogGroup.TRIGGERS -> arrayOf(Analog.LEFT_TRIGGER, Analog.RIGHT_TRIGGER)
                AnalogGroup.ALL -> Analog.values()
            }
        }

        /**
         * `getDebounced` extension for [Controller].
         */
        infix fun Controller.rising(button: Controls): Boolean {
            return this.getDebounced(button)
        }
    }
}