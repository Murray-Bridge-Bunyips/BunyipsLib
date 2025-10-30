package au.edu.sa.mbhs.studentrobotics.bunyipslib.test.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Scheduler;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;

class SchedulerTriggerTest extends SchedulerTests {
    @Test
    void onTrueTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.waitFor(finished::get);
        Gamepad gamepad = new Gamepad();

        gamepad.a = false;
        // Can't access implicit methods
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.A)
                .onTrue(task);
        Scheduler.update();
        assertFalse(task.isActive());
        gamepad.a = true;
        Scheduler.update();
        assertTrue(task.isActive());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isActive());
    }

    @Test
    void onFalseTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.waitFor(finished::get);
        Gamepad gamepad = new Gamepad();

        gamepad.b = true;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.B)
                .onFalse(task);
        Scheduler.update();
        assertFalse(task.isActive());
        gamepad.b = false;
        Scheduler.update();
        assertTrue(task.isActive());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isActive());
    }

    @Test
    void onChangeTest() {
        AtomicBoolean finished = new AtomicBoolean(false);
        Task task = Task.waitFor(finished::get);
        Gamepad gamepad = new Gamepad();

        gamepad.x = true;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.X)
                .onChange(task);
        Scheduler.update();
        assertFalse(task.isActive());
        gamepad.x = false;
        Scheduler.update();
        assertTrue(task.isActive());
        finished.set(true);
        Scheduler.update();
        assertFalse(Scheduler.activeTasks.contains(task));
        assertFalse(task.isActive());
        finished.set(false);
        gamepad.x = true;
        Scheduler.update();
        assertTrue(task.isActive());
        finished.set(true);
        Scheduler.update();
        assertFalse(task.isActive());
    }

    @Test
    void whileTrueRepeatedlyTest() {
        Gamepad gamepad = new Gamepad();
        AtomicInteger inits = new AtomicInteger(0);
        AtomicInteger counter = new AtomicInteger(0);
        Task task = Task.task().init(inits::incrementAndGet).isFinished(() -> counter.incrementAndGet() % 2 == 0)
                .repeatedly();

        gamepad.y = false;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.Y)
                .whileTrue(task);
        Scheduler.update();
        assertEquals(0, inits.get());
        gamepad.y = true;
        Scheduler.update();
        assertEquals(1, inits.get());
        Scheduler.update();
        assertEquals(1, inits.get());
        Scheduler.update();
        assertEquals(2, inits.get());
        gamepad.y = false;
        Scheduler.update();
        assertEquals(2, inits.get());
    }

    @Test
    void whileTrueLambdaRunTest() {
        Gamepad gamepad = new Gamepad();
        AtomicInteger counter = new AtomicInteger(0);
        Task task = new Lambda(counter::incrementAndGet)
                .repeatedly();

        gamepad.left_stick_button = false;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.LEFT_STICK_BUTTON)
                .whileTrue(task);
        Scheduler.update();
        assertEquals(0, counter.get());
        gamepad.left_stick_button = true;
        Scheduler.update();
        assertEquals(1, counter.get());
        Scheduler.update();
        assertEquals(2, counter.get());
        gamepad.left_stick_button = false;
        Scheduler.update();
        assertEquals(2, counter.get());
    }

    @Test
    void whileTrueOnceTest() {
        AtomicInteger startCounter = new AtomicInteger(0);
        AtomicInteger endCounter = new AtomicInteger(0);
        Task task = Task.task().init(startCounter::incrementAndGet).onFinish(endCounter::incrementAndGet);
        Gamepad gamepad = new Gamepad();

        gamepad.back = false;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.BACK)
                .whileTrue(task);
        Scheduler.update();
        assertEquals(0, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.back = true;
        Scheduler.update();
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.back = false;
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(1, endCounter.get());
    }


    @Test
    void toggleOnTrueTest() {
        AtomicInteger startCounter = new AtomicInteger(0);
        AtomicInteger endCounter = new AtomicInteger(0);
        Task task = Task.task().init(startCounter::incrementAndGet).onFinish(endCounter::incrementAndGet);
        Gamepad gamepad = new Gamepad();

        gamepad.a = false;
        new Scheduler.GamepadTrigger(GamepadUser.ONE, gamepad).button(Controls.A)
                .toggleOnTrue(task);
        Scheduler.update();
        assertEquals(0, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.a = true;
        Scheduler.update();
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.a = false;
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.a = true;
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(1, endCounter.get());
    }

    @Test
    void cancelWhenActiveTest() {
        AtomicInteger startCounter = new AtomicInteger(0);
        AtomicInteger endCounter = new AtomicInteger(0);
        Gamepad gamepad = new Gamepad();
        Task task = Task.task().init(startCounter::incrementAndGet).onFinish(endCounter::incrementAndGet)
                .until(() -> gamepad.b);

        gamepad.b = false;
        Scheduler.schedule(task);
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(0, endCounter.get());
        gamepad.b = true;
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(1, endCounter.get());
        Scheduler.update();
        assertEquals(1, startCounter.get());
        assertEquals(1, endCounter.get());
    }

    @Test
    void triggerCompositionTest() {
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        gamepad1.dpad_down = true;
        gamepad2.dpad_up = false;

        Scheduler.Trigger button1 = Scheduler.on(new Scheduler.GamepadTrigger.ButtonBind(gamepad1, Controls.DPAD_DOWN));
        Scheduler.Trigger button2 = Scheduler.on(new Scheduler.GamepadTrigger.ButtonBind(gamepad1, Controls.DPAD_UP));

        assertFalse(button1.and(button2).getAsBoolean());
        assertTrue(button1.or(button2).getAsBoolean());
        assertFalse(button1.negate().getAsBoolean());
        assertTrue(button1.and(button2.negate()).getAsBoolean());
    }

    @Test
    void triggerCompositionSupplierTest() {
        Gamepad gamepad = new Gamepad();
        BooleanSupplier booleanSupplier = () -> false;

        gamepad.left_bumper = true;
        Scheduler.Trigger button1 = Scheduler.on(new Scheduler.GamepadTrigger.ButtonBind(gamepad, Controls.LEFT_BUMPER));

        assertFalse(button1.and(booleanSupplier).getAsBoolean());
        assertTrue(button1.or(booleanSupplier).getAsBoolean());
    }

    @Test
    void debounceTest() throws InterruptedException {
        AtomicBoolean currBool = new AtomicBoolean(false);
        AtomicBoolean scheduled = new AtomicBoolean(false);

        Scheduler.on(currBool::get).withActiveDelay(Milliseconds.of(500))
                .onTrue(new Lambda(() -> scheduled.set(true)));

        Scheduler.update();
        assertFalse(scheduled.get());
        currBool.set(true);
        for (int i = 0; i < 200; i++) {
            Scheduler.update();
            assertFalse(scheduled.get());
            Thread.sleep(1);
        }

        Thread.sleep(350);

        Scheduler.update();
        assertTrue(scheduled.get());
    }

    @Test
    void booleanSupplierTest() {
        AtomicBoolean currBool = new AtomicBoolean(false);
        Scheduler.Trigger bool = Scheduler.on(currBool::get);

        assertFalse(bool.getAsBoolean());
        currBool.set(true);
        assertTrue(bool.getAsBoolean());
    }
}
