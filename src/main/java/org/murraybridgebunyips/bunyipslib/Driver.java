package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * A wrapper around a Gamepad object that provides a Controller interface and custom input calculations.
 * @author Lucas Bubner, 2024
 */
public class Driver {
    private final Gamepad gamepad;
    private final HashMap<Controller, Predicate<Boolean>> buttons = new HashMap<>();
    private final HashMap<Controller.Analog, Function<Double, Double>> axes = new HashMap<>();

    /**
     * A function that returns the input value as-is.
     */
    public static final Function<Double, Double> LINEAR = x -> x;
    /**
     * A function that squares the input value and keeps the sign.
     */
    public static final Function<Double, Double> SQUARE = x -> Math.copySign(x * x, x);
    /**
     * A function that cubes the input value and keeps the sign.
     */
    public static final Function<Double, Double> CUBE = x -> Math.copySign(x * x * x, x);
    /**
     * A function that takes the absolute value of the input.
     */
    public static final Function<Double, Double> ABS = Math::abs;
    /**
     * A function that negates the input value.
     */
    public static final Function<Double, Double> NEGATE = x -> -x;

    /**
     * Groups of buttons that can be customised.
     */
    public enum ButtonGroup {
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
    public enum AnalogGroup {
        /**
         * Both analog sticks
         */
        ANALOG_STICKS,
        /**
         * Both triggers
         */
        TRIGGERS,
        /**
         * All analog inputs
         */
        ALL
    }

    private Controller[] getButtons(ButtonGroup group) {
        switch (group) {
            case BUMPERS:
                return new Controller[] {Controller.LEFT_BUMPER, Controller.RIGHT_BUMPER};
            case DPAD:
                return new Controller[] {Controller.DPAD_UP, Controller.DPAD_DOWN, Controller.DPAD_LEFT, Controller.DPAD_RIGHT};
            case BUTTONS:
                return new Controller[] {Controller.A, Controller.B, Controller.X, Controller.Y};
            case SPECIAL:
                return new Controller[] {Controller.START, Controller.BACK, Controller.LEFT_STICK_BUTTON, Controller.RIGHT_STICK_BUTTON};
            case ALL:
                return Controller.values();
            default:
                return new Controller[0];
        }
    }

    private Controller.Analog[] getAxes(AnalogGroup group) {
        switch (group) {
            case ANALOG_STICKS:
                return new Controller.Analog[] {Controller.Analog.LEFT_STICK_X, Controller.Analog.LEFT_STICK_Y, Controller.Analog.RIGHT_STICK_X, Controller.Analog.RIGHT_STICK_Y};
            case TRIGGERS:
                return new Controller.Analog[] {Controller.Analog.LEFT_TRIGGER, Controller.Analog.RIGHT_TRIGGER};
            case ALL:
                return Controller.Analog.values();
            default:
                return new Controller.Analog[0];
        }
    }

    /**
     * Customise how a button is read.
     * @param button The button to customise
     * @param predicate The custom function to use based on the button's value
     * @return this
     */
    public Driver set(Controller button, @Nullable Predicate<Boolean> predicate) {
        if (predicate == null) {
            buttons.remove(button);
            return this;
        }
        buttons.put(button, predicate);
        return this;
    }

    /**
     * Customise how an axis is read.
     * @param axis The axis to customise
     * @param function The custom function to use based on the axis's value
     * @return this
     */
    public Driver set(Controller.Analog axis, @Nullable Function<Double, Double> function) {
        if (function == null) {
            axes.remove(axis);
            return this;
        }
        axes.put(axis, function);
        return this;
    }

    /**
     * Customise how a group of buttons is read.
     * @param group The group of buttons to customise
     * @param predicate The custom function to use based on the group's value
     * @return this
     */
    public Driver set(ButtonGroup group, @Nullable Predicate<Boolean> predicate) {
        for (Controller button : getButtons(group)) {
            set(button, predicate);
        }
        return this;
    }

    /**
     * Customise how a group of axes is read.
     * @param group The group of axes to customise
     * @param function The custom function to use based on the group's value
     * @return this
     */
    public Driver set(AnalogGroup group, @Nullable Function<Double, Double> function) {
        for (Controller.Analog axis : getAxes(group)) {
            set(axis, function);
        }
        return this;
    }

    /**
     * Create a new Driver.
     * @param gamepad The Gamepad to wrap (gamepad1, gamepad2)
     */
    public Driver(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Get the value of a button.
     * @param button The button to get the value of
     * @return The value of the button
     */
    public boolean get(Controller button) {
        boolean isPressed = Controller.isSelected(gamepad, button);
        Predicate<Boolean> predicate = buttons.get(button);
        return predicate == null ? isPressed : predicate.test(isPressed);
    }

    /**
     * Get the value of an axis.
     * @param axis The axis to get the value of
     * @return The value of the axis
     */
    public double get(Controller.Analog axis) {
        double value = Controller.Analog.get(gamepad, axis);
        Function<Double, Double> function = axes.get(axis);
        return function == null ? value : function.apply(value);
    }

    /**
     * @return left_stick_x
     */
    public double left_stick_x() {
        return get(Controller.Analog.LEFT_STICK_X);
    }

    /**
     * @return left_stick_x
     */
    public double lx() {
        return left_stick_x();
    }

    /**
     * @return left_stick_y
     */
    public double left_stick_y() {
        return get(Controller.Analog.LEFT_STICK_Y);
    }

    /**
     * @return left_stick_y
     */
    public double ly() {
        return left_stick_y();
    }

    /**
     * @return left_stick_button
     */
    public boolean left_stick_button() {
        return get(Controller.LEFT_STICK_BUTTON);
    }

    /**
     * @return left_stick_button
     */
    public boolean lsb() {
        return left_stick_button();
    }

    /**
     * @return right_stick_x
     */
    public double right_stick_x() {
        return get(Controller.Analog.RIGHT_STICK_X);
    }

    /**
     * @return right_stick_x
     */
    public double rx() {
        return right_stick_x();
    }

    /**
     * @return right_stick_y
     */
    public double right_stick_y() {
        return get(Controller.Analog.RIGHT_STICK_Y);
    }

    /**
     * @return right_stick_y
     */
    public double ry() {
        return right_stick_y();
    }

    /**
     * @return right_stick_button
     */
    public boolean right_stick_button() {
        return get(Controller.RIGHT_STICK_BUTTON);
    }

    /**
     * @return right_stick_button
     */
    public boolean rsb() {
        return right_stick_button();
    }

    /**
     * @return left_trigger
     */
    public double left_trigger() {
        return get(Controller.Analog.LEFT_TRIGGER);
    }

    /**
     * @return left_trigger
     */
    public double lt() {
        return left_trigger();
    }

    /**
     * @return right_trigger
     */
    public double right_trigger() {
        return get(Controller.Analog.RIGHT_TRIGGER);
    }

    /**
     * @return right_trigger
     */
    public double rt() {
        return right_trigger();
    }

    /**
     * @return left_bumper
     */
    public boolean left_bumper() {
        return get(Controller.LEFT_BUMPER);
    }

    /**
     * @return left_bumper
     */
    public boolean lb() {
        return left_bumper();
    }

    /**
     * @return right_bumper
     */
    public boolean right_bumper() {
        return get(Controller.RIGHT_BUMPER);
    }

    /**
     * @return right_bumper
     */
    public boolean rb() {
        return right_bumper();
    }

    /**
     * @return dpad_up
     */
    public boolean dpad_up() {
        return get(Controller.DPAD_UP);
    }

    /**
     * @return dpad_up
     */
    public boolean du() {
        return dpad_up();
    }

    /**
     * @return dpad_down
     */
    public boolean dpad_down() {
        return get(Controller.DPAD_DOWN);
    }

    /**
     * @return dpad_down
     */
    public boolean dd() {
        return dpad_down();
    }

    /**
     * @return dpad_left
     */
    public boolean dpad_left() {
        return get(Controller.DPAD_LEFT);
    }

    /**
     * @return dpad_left
     */
    public boolean dl() {
        return dpad_left();
    }

    /**
     * @return dpad_right
     */
    public boolean dpad_right() {
        return get(Controller.DPAD_RIGHT);
    }

    /**
     * @return dpad_right
     */
    public boolean dr() {
        return dpad_right();
    }

    /**
     * @return a
     */
    public boolean a() {
        return get(Controller.A);
    }

    /**
     * @return b
     */
    public boolean b() {
        return get(Controller.B);
    }

    /**
     * @return x
     */
    public boolean x() {
        return get(Controller.X);
    }

    /**
     * @return y
     */
    public boolean y() {
        return get(Controller.Y);
    }

    /**
     * @return back
     */
    public boolean back() {
        return get(Controller.BACK);
    }

    /**
     * @return start
     */
    public boolean start() {
        return get(Controller.START);
    }
}
