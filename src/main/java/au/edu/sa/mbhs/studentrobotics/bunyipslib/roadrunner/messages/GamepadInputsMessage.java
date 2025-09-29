package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * RoadRunner v1.0 logging message for AdvantageScope-compatible Joystick inputs.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class GamepadInputsMessage {
    /**
     * Logged pressed buttons.
     */
    public boolean[] buttons;
    /**
     * Logged values of analog axes.
     */
    public double[] axes;
    /**
     * Encoded D-pad value.
     */
    public int[] povs;

    @SuppressWarnings("MissingJavadoc")
    public GamepadInputsMessage(Gamepad gamepad) {
        buttons = new boolean[]{
                gamepad.a, gamepad.b, gamepad.x, gamepad.y,
                gamepad.left_bumper, gamepad.right_bumper,
                gamepad.back, gamepad.start,
                gamepad.left_stick_button, gamepad.right_stick_button
        };
        axes = new double[]{
                gamepad.left_stick_x, gamepad.left_stick_y,
                gamepad.left_trigger, gamepad.right_trigger,
                gamepad.right_stick_x, gamepad.right_stick_y
        };
        povs = new int[]{getPov(gamepad)};
    }

    /**
     * Gets the current POV of the D-pad on a gamepad.
     *
     * @param gamepad desired gamepad
     * @return the POV value in degrees
     */
    public static int getPov(Gamepad gamepad) {
        int pov = -1;
        boolean up = gamepad.dpad_up, left = gamepad.dpad_left, down = gamepad.dpad_down, right = gamepad.dpad_right;
        if (up && left)
            pov = 315;
        else if (up && right)
            pov = 45;
        else if (up)
            pov = 0;
        else if (right && down)
            pov = 135;
        else if (right)
            pov = 90;
        else if (left && down)
            pov = 225;
        else if (down)
            pov = 180;
        else if (left)
            pov = 270;
        return pov;
    }
}
