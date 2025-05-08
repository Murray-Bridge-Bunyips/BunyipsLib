package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.function.Predicate;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.UnaryFunction;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.logic.Condition;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;

/**
 * A wrapper around a {@link Gamepad} object that provides a {@link Controls} interface and custom input calculations.
 * These gamepad objects are used natively in {@link BunyipsOpMode}, and are the drop-in replacements for {@code gamepad1} and {@code gamepad2}.
 * Controller instances are injected into the FtcEventLoop through {@link #inject(Controller, Controller)}.
 *
 * @author Lucas Bubner, 2024
 * @see BunyipsOpMode
 * @since 7.3.0
 */
public class Controller extends Gamepad {
    /**
     * The user defined designated user for this gamepad controller.
     * <p>
     * See the constructor notes for more information.
     */
    public final GamepadUser designatedUser;
    private final HashMap<Controls, Predicate<Boolean>> buttons = new HashMap<>();
    private final HashMap<Controls.Analog, UnaryFunction> axes = new HashMap<>();
    private final HashMap<Controls, Boolean> debounces = new HashMap<>();
    /**
     * Shorthand for left_stick_x
     */
    public volatile float lsx;
    /**
     * Shorthand for left_stick_y
     */
    public volatile float lsy;
    /**
     * Shorthand for right_stick_x
     */
    public volatile float rsx;
    /**
     * Shorthand for right_stick_y
     */
    public volatile float rsy;
    /**
     * Shorthand for left_trigger
     */
    public volatile float lt;
    /**
     * Shorthand for right_trigger
     */
    public volatile float rt;
    /**
     * Shorthand for left_bumper
     */
    public volatile boolean lb;
    /**
     * Shorthand for right_bumper
     */
    public volatile boolean rb;
    /**
     * Shorthand for dpad_up
     */
    public volatile boolean du;
    /**
     * Shorthand for dpad_down
     */
    public volatile boolean dd;
    /**
     * Shorthand for dpad_left
     */
    public volatile boolean dl;
    /**
     * Shorthand for dpad_right
     */
    public volatile boolean dr;
    /**
     * Shorthand for left_stick_button
     */
    public volatile boolean lsb;
    /**
     * Shorthand for right_stick_button
     */
    public volatile boolean rsb;

    /**
     * Create a new Controller to manage.
     * <p>
     * The Controller instance must be injected into the {@link EventLoopManager} through the static method {@link #inject(Controller, Controller)}.
     *
     * @param designatedUser The user this gamepad is designated to. Normally, accessing the {@code gamepad.getUser()}
     *                       method can return null if the gamepad has not been "activated" via the controller by
     *                       the user doing Start + A or B. This behaviour is a strange oddity of the SDK but leads
     *                       to unexpected behaviour for users extracting controller data in init.
     *                       This field is used to re-expose as the {@link #designatedUser} public field, which ensures
     *                       that a user object can be retrieved without risk of it being null. Note that the presence
     *                       of this field does not impact the {@link #user} field on the actual gamepad, as it is
     *                       being parsed directly from the SDK to ensure consistent behaviour and eliminate risk
     *                       of race conditions. A try-getter, {@link #tryGetUser} exists for convenience.
     */
    public Controller(@NonNull GamepadUser designatedUser) {
        this.designatedUser = designatedUser;
    }

    /**
     * Injects two instances of Controller into the event loop manager and OpMode, causing the underlying gamepad objects
     * to be instances of a Controller, extending Gamepad.
     * <p>
     * BunyipsOpMode automatically calls this method as part of a pre-initialisation hook.
     * <p>
     * e.g. Injecting new unmanaged Controller objects into all current and future run OpModes of this power cycle:
     * {@code Controller.inject(new Controller(GamepadUser.ONE), new Controller(GamepadUser.TWO);}
     *
     * @param gamepad1 gamepad user 1 to inject
     * @param gamepad2 gamepad user 2 to inject
     */
    public static void inject(@NonNull Controller gamepad1, @NonNull Controller gamepad2) {
        // We could do this one controller at a time and check designatedUser for allocation but
        // realistically this method should never be called for just one controller.
        try {
            Field felh = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            felh.setAccessible(true);
            Field elm = FtcEventLoopHandler.class.getDeclaredField("eventLoopManager");
            elm.setAccessible(true);
            Field gamepads = EventLoopManager.class.getDeclaredField("opModeGamepads");
            gamepads.setAccessible(true);
            gamepads.set(elm.get(felh.get(BunyipsLib.getFtcEventLoop())), new Gamepad[]{gamepad1, gamepad2});
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("Could not access internal event loop fields, this shouldn't happen!", e);
        }
        // Update any stale instances
        BunyipsLib.getOpMode().gamepad1 = gamepad1;
        BunyipsLib.getOpMode().gamepad2 = gamepad2;
    }

    /**
     * Attempts to get the user of this controller.
     * <p>
     * If a user cannot be found directly and this gamepad is a Controller instance, the {@link #designatedUser}
     * will be returned. Otherwise, if no user can be found, null is returned.
     *
     * @param gamepad the gamepad to get from
     * @return the user of the controller, if available by user getter and designated user means, else null
     */
    @Nullable
    public static GamepadUser tryGetUser(@NonNull Gamepad gamepad) {
        GamepadUser user = gamepad.getUser();
        if (user == null && gamepad instanceof Controller ctrl) {
            return ctrl.designatedUser;
        }
        return user;
    }

    /**
     * Copies new state into this gamepad. This method is where new data is added into this gamepad.
     *
     * @param gamepad state to be copied from
     */
    @Override
    public void copy(Gamepad gamepad) {
        fromByteArray(gamepad.toByteArray());
    }

    /**
     * Parse gamepad info and assign it to the public fields. This method is the primary updater called periodically.
     *
     * @param byteArray byte array containing valid gamepad data
     */
    @Override
    public void fromByteArray(byte[] byteArray) {
        // Extracted from Gamepad, lifted here to assign custom functions
        final int BUFFER_SIZE = 65;
        if (byteArray.length < BUFFER_SIZE) {
            throw new RuntimeException("Expected buffer of at least " + BUFFER_SIZE + " bytes, received " + byteArray.length);
        }
        ByteBuffer byteBuffer = getReadBuffer(byteArray);
        int buttons;
        byte version = byteBuffer.get();
        // Extract version 1 values
        if (version >= 1) {
            id = byteBuffer.getInt();
            timestamp = byteBuffer.getLong();
            left_stick_x = transform(byteBuffer.getFloat(), Controls.Analog.LEFT_STICK_X);
            left_stick_y = transform(byteBuffer.getFloat(), Controls.Analog.LEFT_STICK_Y);
            right_stick_x = transform(byteBuffer.getFloat(), Controls.Analog.RIGHT_STICK_X);
            right_stick_y = transform(byteBuffer.getFloat(), Controls.Analog.RIGHT_STICK_Y);
            left_trigger = transform(byteBuffer.getFloat(), Controls.Analog.LEFT_TRIGGER);
            right_trigger = transform(byteBuffer.getFloat(), Controls.Analog.RIGHT_TRIGGER);
            buttons = byteBuffer.getInt();
            touchpad_finger_1 = (buttons & 0x20000) != 0;
            touchpad_finger_2 = (buttons & 0x10000) != 0;
            touchpad = (buttons & 0x08000) != 0;
            left_stick_button = transform((buttons & 0x04000) != 0, Controls.LEFT_STICK_BUTTON);
            right_stick_button = transform((buttons & 0x02000) != 0, Controls.RIGHT_STICK_BUTTON);
            dpad_up = transform((buttons & 0x01000) != 0, Controls.DPAD_UP);
            dpad_down = transform((buttons & 0x00800) != 0, Controls.DPAD_DOWN);
            dpad_left = transform((buttons & 0x00400) != 0, Controls.DPAD_LEFT);
            dpad_right = transform((buttons & 0x00200) != 0, Controls.DPAD_RIGHT);
            a = transform((buttons & 0x00100) != 0, Controls.A);
            b = transform((buttons & 0x00080) != 0, Controls.B);
            x = transform((buttons & 0x00040) != 0, Controls.X);
            y = transform((buttons & 0x00020) != 0, Controls.Y);
            guide = (buttons & 0x00010) != 0;
            start = transform((buttons & 0x00008) != 0, Controls.START);
            back = transform((buttons & 0x00004) != 0, Controls.BACK);
            left_bumper = transform((buttons & 0x00002) != 0, Controls.LEFT_BUMPER);
            right_bumper = transform((buttons & 0x00001) != 0, Controls.RIGHT_BUMPER);
        }
        // Extract version 2 values
        if (version >= 2) {
            user = byteBuffer.get();
        }
        // Extract version 3 values
        if (version >= 3) {
            type = Type.values()[byteBuffer.get()];
        }
        if (version >= 4) {
            byte v4TypeValue = byteBuffer.get();
            if (v4TypeValue < Type.values().length) {
                // Yes, this will replace the version 3 value. That is a good thing, since the version 3
                // value was not forwards-compatible.
                type = Type.values()[v4TypeValue];
            } // Else, we don't know what the number means, so we just stick with the value we got from the v3 type field
        }
        if (version >= 5) {
            touchpad_finger_1_x = byteBuffer.getFloat();
            touchpad_finger_1_y = byteBuffer.getFloat();
            touchpad_finger_2_x = byteBuffer.getFloat();
            touchpad_finger_2_y = byteBuffer.getFloat();
        }
        updateButtonAliases();
    }

    private boolean transform(boolean sdk, Controls button) {
        Predicate<Boolean> predicate = buttons.get(button);
        return predicate == null ? sdk : predicate.test(sdk);
    }

    private float transform(float sdk, Controls.Analog axis) {
        UnaryFunction function = axes.get(axis);
        return function == null ? sdk : (float) function.apply(sdk);
    }

    @Override
    protected void updateButtonAliases() {
        super.updateButtonAliases();
        // Assign public field aliases
        lsx = left_stick_x;
        lsy = left_stick_y;
        rsx = right_stick_x;
        rsy = right_stick_y;
        lt = left_trigger;
        rt = right_trigger;
        lb = left_bumper;
        rb = right_bumper;
        du = dpad_up;
        dd = dpad_down;
        dl = dpad_left;
        dr = dpad_right;
        lsb = left_stick_button;
        rsb = right_stick_button;
    }

    /**
     * Customise how a button is read.
     *
     * @param button    The button to customise
     * @param predicate The custom function to use based on the button's value
     * @return this
     */
    @NonNull
    public Controller set(@NonNull Controls button, @Nullable Predicate<Boolean> predicate) {
        if (predicate == null) {
            buttons.remove(button);
            return this;
        }
        if (button != Controls.NONE)
            buttons.put(button, predicate);
        // Propagate an update
        fromByteArray(toByteArray());
        return this;
    }

    /**
     * Customise how an axis is read.
     *
     * @param axis     The axis to customise
     * @param function The custom function to use based on the axis's value
     * @return this
     */
    @NonNull
    public Controller set(@NonNull Controls.Analog axis, @Nullable UnaryFunction function) {
        if (function == null) {
            axes.remove(axis);
            return this;
        }
        axes.put(axis, function);
        // Propagate an update
        fromByteArray(toByteArray());
        return this;
    }

    /**
     * Customise how a group of buttons is read.
     *
     * @param group     The group of buttons to customise
     * @param predicate The custom function to use based on the group's value
     * @return this
     */
    @NonNull
    public Controller set(@NonNull Controls.ButtonGroup group, @Nullable Predicate<Boolean> predicate) {
        for (Controls button : Controls.getButtons(group)) {
            set(button, predicate);
        }
        return this;
    }

    /**
     * Customise how a group of axes is read.
     *
     * @param group    The group of axes to customise
     * @param function The custom function to use based on the group's value
     * @return this
     */
    @NonNull
    public Controller set(@NonNull Controls.AnalogGroup group, @Nullable UnaryFunction function) {
        for (Controls.Analog axis : Controls.getAxes(group)) {
            set(axis, function);
        }
        return this;
    }

    /**
     * Check if a button is currently pressed on a gamepad, with debounce to ignore a press that was already detected
     * upon the <b>first call of this function and button</b>. This is an implementation of rising edge detection, but also
     * applies a check for the initial state of the button, making it useful for task toggles.
     * <p>
     * See the {@link Condition} class for more boolean state management.
     *
     * @param button The button to check
     * @return True if the button is pressed and not debounced by the definition of this method
     */
    public boolean getDebounced(@NonNull Controls button) {
        boolean buttonPressed = Controls.isSelected(this, button);
        boolean pressedPreviously = Boolean.TRUE.equals(debounces.getOrDefault(button, buttonPressed));
        if (buttonPressed && !pressedPreviously) {
            debounces.put(button, true);
            return true;
        } else if (!buttonPressed) {
            debounces.put(button, false);
        }
        return false;
    }

    /**
     * Call to reset the initial debounce state of {@link #getDebounced(Controls)}, allowing further calls to
     * this method to capture the initial state of the button again. For implementations that do not call this method,
     * the {@link #getDebounced(Controls)} method will operate as a simple rising edge detector.
     *
     * @param button The button to reset the debounce state of
     */
    public void resetDebounce(@NonNull Controls button) {
        debounces.remove(button);
    }
}
