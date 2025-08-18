package au.edu.sa.mbhs.studentrobotics.bunyipslib.test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;

/**
 * Tests for {@link au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller}.
 *
 * @author Lucas Bubner, 2025
 */
class ControllerTest {
    Gamepad target;
    Controller controller;

    @BeforeEach
    void setUp() {
        target = new Gamepad();
        controller = new Controller(GamepadUser.ONE);
    }

    @Test
    void testState() {
        target.a = true;
        target.dpad_left = true;
        target.left_stick_x = (float) Math.random();
        target.right_stick_x = (float) Math.random();
        controller.copy(target);
        assertArrayEquals(controller.toByteArray(), target.toByteArray());
    }

    @Test
    void testLinkedAlias() {
        target.left_stick_x = 0.5f;
        controller.copy(target);
        assertEquals(0.5f, controller.lsx);
    }

    @Test
    void testUnaryFunction() {
        controller.set(Controls.Analog.LEFT_STICK_X, (v) -> v * 2);
        target.left_stick_x = 0.5f;
        controller.copy(target);
        assertEquals(1.0f, controller.lsx);
    }

    @Test
    void testPredicate() {
        controller.set(Controls.A, (v) -> !v);
        target.a = true;
        controller.copy(target);
        assertFalse(controller.a);
        target.a = false;
        controller.copy(target);
        assertTrue(controller.a);
    }

    @Test
    void testDebounce() {
        target.x = false;
        controller.copy(target);
        assertFalse(controller.getDebounced(Controls.X)); // initial condition
        target.x = true;
        controller.copy(target);
        assertTrue(controller.getDebounced(Controls.X)); // actual debounce state
        assertFalse(controller.getDebounced(Controls.X));
        target.x = false;
        controller.copy(target);
        assertFalse(controller.getDebounced(Controls.X));
    }

    @Test
    void testSDKRisingEdge() {
        target.a = true;
        controller.copy(target);
        assertTrue(controller.aWasPressed());
        assertFalse(controller.aWasPressed());
        controller.copy(target);
        assertFalse(controller.aWasPressed());
    }

    @Test
    void testSDKFallingEdge() {
        target.dpad_left = true;
        controller.copy(target);
        target.dpad_left = false;
        assertFalse(controller.dpadLeftWasReleased());
        controller.copy(target);
        assertTrue(controller.dpadLeftWasReleased());
        assertFalse(controller.dpadLeftWasReleased());
        controller.copy(target);
        assertFalse(controller.dpadLeftWasReleased());
    }
}
