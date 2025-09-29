package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * RoadRunner v1.0 logging message for Controller inputs.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class GamepadInputsMessage {
    /**
     * the timestamp this message was created
     */
    public long timestamp;
    /**
     * relative timestamp from the gamepad when an event was detected
     */
    public long relativeTimestamp;
    /**
     * left analog stick horizontal axis
     */
    public double left_stick_x;
    /**
     * left analog stick vertical axis
     */
    public double left_stick_y;
    /**
     * right analog stick horizontal axis
     */
    public double right_stick_x;
    /**
     * right analog stick vertical axis
     */
    public double right_stick_y;
    /**
     * dpad up
     */
    public boolean dpad_up;
    /**
     * dpad down
     */
    public boolean dpad_down;
    /**
     * dpad left
     */
    public boolean dpad_left;
    /**
     * dpad right
     */
    public boolean dpad_right;
    /**
     * button a
     */
    public boolean a;
    /**
     * button b
     */
    public boolean b;
    /**
     * button x
     */
    public boolean x;
    /**
     * button y
     */
    public boolean y;
    /**
     * button guide - often the large button in the middle of the controller. The OS may
     * capture this button before it is sent to the app; in which case you'll never
     * receive it.
     */
    public boolean guide;
    /**
     * button start
     */
    public boolean start;
    /**
     * button back
     */
    public boolean back;
    /**
     * button left bumper
     */
    public boolean left_bumper;
    /**
     * button right bumper
     */
    public boolean right_bumper;
    /**
     * left stick button
     */
    public boolean left_stick_button;
    /**
     * right stick button
     */
    public boolean right_stick_button;
    /**
     * left trigger
     */
    public double left_trigger;
    /**
     * right trigger
     */
    public double right_trigger;
    /**
     * PS4 Support - touchpad
     */
    public boolean touchpad;
    /**
     * PS4 Support - finger 1 detection
     */
    public boolean touchpad_finger_1;
    /**
     * PS4 Support - finger 2 detection
     */
    public boolean touchpad_finger_2;
    /**
     * PS4 Support - finger 1 x position
     */
    public double touchpad_finger_1_x;
    /**
     * PS4 Support - finger 1 y position
     */
    public double touchpad_finger_1_y;
    /**
     * PS4 Support - finger 2 x position
     */
    public double touchpad_finger_2_x;
    /**
     * PS4 Support - finger 2 y position
     */
    public double touchpad_finger_2_y;

    @SuppressWarnings("MissingJavadoc")
    public GamepadInputsMessage(Gamepad gamepad) {
        timestamp = System.nanoTime();
        relativeTimestamp = gamepad.timestamp;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        guide = gamepad.guide;
        start = gamepad.start;
        back = gamepad.back;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        touchpad = gamepad.touchpad;
        touchpad_finger_1 = gamepad.touchpad_finger_1;
        touchpad_finger_2 = gamepad.touchpad_finger_2;
        touchpad_finger_1_x = gamepad.touchpad_finger_1_x;
        touchpad_finger_1_y = gamepad.touchpad_finger_1_y;
        touchpad_finger_2_x = gamepad.touchpad_finger_2_x;
        touchpad_finger_2_y = gamepad.touchpad_finger_2_y;
    }
}
