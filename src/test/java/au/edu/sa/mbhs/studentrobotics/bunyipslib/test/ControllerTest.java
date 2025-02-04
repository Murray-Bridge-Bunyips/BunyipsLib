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
 * @author Lucas Bubner, 2024
 */
class ControllerTest {
    Gamepad gamepad;
    Controller controller;

    @BeforeEach
    void setUp() {
        gamepad = new Gamepad();
        controller = new Controller(gamepad, GamepadUser.ONE);
    }

    @Test
    void testState() {
        assertEquals(controller.sdk, gamepad);
        gamepad.a = true;
        controller.update();
        assertArrayEquals(controller.toByteArray(), gamepad.toByteArray());
    }

    @Test
    void testLinkedAlias() {
        gamepad.left_stick_x = 0.5f;
        controller.update();
        assertEquals(0.5f, controller.lsx);
    }

    @Test
    void testUnaryFunction() {
        controller.set(Controls.Analog.LEFT_STICK_X, (v) -> v * 2);
        gamepad.left_stick_x = 0.5f;
        controller.update();
        assertEquals(1.0f, controller.lsx);
    }

    @Test
    void testPredicate() {
        controller.set(Controls.A, (v) -> !v);
        gamepad.a = true;
        controller.update();
        assertFalse(controller.a);
        gamepad.a = false;
        controller.update();
        assertTrue(controller.a);
    }

    @Test
    void testDebounce() {
        gamepad.x = false;
        controller.update();
        assertFalse(controller.getDebounced(Controls.X)); // initial condition
        gamepad.x = true;
        controller.update();
        assertTrue(controller.getDebounced(Controls.X)); // actual debounce state
        assertFalse(controller.getDebounced(Controls.X));
        gamepad.x = false;
        controller.update();
        assertFalse(controller.getDebounced(Controls.X));
    }
}
