package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import android.graphics.Color;
import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Watchdog;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * A collection of {@link BunyipsOpMode} debug features intended for use in a testing environment, including
 * a controller-based kill switch, robot pickup detection, and customisable auto-halt conditions.
 *
 * @author Lucas Bubner, 2024
 * @since 4.1.0
 */
@Config
public class DebugMode extends BunyipsComponent implements Runnable {
    /**
     * The threshold at which {@link #whenRobotRolled(IMU, TriggerAction)} will trigger for the pitch and roll angles.
     */
    public static double ROBOT_PITCH_ROLL_THRESHOLD_DEGREES = 5;

    private final ArrayList<Pair<BooleanSupplier, TriggerAction>> actions = new ArrayList<>();
    private final ArrayList<Watchdog> watchdogs = new ArrayList<>();
    private boolean hasResetWatchdogs = false;
    private int max = -1;

    /**
     * Enable debug mode.
     */
    public DebugMode() {
        require(opMode).onActiveLoop(this);
        Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode.");
    }

    /**
     * Construct a new instance of DebugMode.
     *
     * @return instance of this to use with a builder pattern
     */
    public static DebugMode enable() {
        return new DebugMode();
    }

    /**
     * Add a custom actionable condition.
     *
     * @param stopCondition the condition to periodically check
     * @param action        the action to take if the condition is met
     * @return this
     */
    public DebugMode when(BooleanSupplier stopCondition, TriggerAction action) {
        actions.add(new Pair<>(stopCondition, action));
        return this;
    }

    /**
     * Utility to use the IMU to detect roll or pitch beyond {@link #ROBOT_PITCH_ROLL_THRESHOLD_DEGREES}.
     *
     * @param imu    the IMU to use for detection
     * @param action the action to take on a roll/pitch violation
     * @return this
     */
    public DebugMode whenRobotRolled(IMU imu, TriggerAction action) {
        actions.add(new Pair<>(() -> {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double roll = angles.getRoll(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            return Math.abs(roll) >= ROBOT_PITCH_ROLL_THRESHOLD_DEGREES || Math.abs(pitch) >= ROBOT_PITCH_ROLL_THRESHOLD_DEGREES;
        }, action));
        return this;
    }

    /**
     * Utility to use a "killbind" where pressing the {@code START} and {@code BACK} buttons simultaneously on
     * either gamepad runs the action.
     *
     * @param action the action to take on pressing START+BACK on either gamepad1 or gamepad2
     * @return this
     */
    public DebugMode whenGamepadKillSwitch(TriggerAction action) {
        actions.add(new Pair<>(() ->
                (require(opMode).gamepad1.back && opMode.gamepad1.start) || (opMode.gamepad2.back && opMode.gamepad2.start),
                action
        ));
        return this;
    }

    /**
     * Create a new watchdog that can be updated via index through {@link #getWatchdogs()}.
     * If this watchdog dies, the action will be executed.
     *
     * @param timeout the timeout before this watchdog dies
     * @param action  the action to take on timeout
     * @return this
     */
    public DebugMode whenWatchdogExpires(Measure<Time> timeout, TriggerAction action) {
        Watchdog w = new Watchdog(() -> {
            // no-op, we check for running state not a callback
        }, 50, (long) timeout.in(Milliseconds), TimeUnit.MILLISECONDS);
        watchdogs.add(w);
        actions.add(new Pair<>(() -> !w.isRunning(), action));
        return this;
    }

    /**
     * Get the added watchdogs via {@link #whenWatchdogExpires(Measure, TriggerAction)}. These will be added
     * by index of timeout calls.
     *
     * @return the added watchdogs
     */
    public Watchdog[] getWatchdogs() {
        return watchdogs.toArray(new Watchdog[0]);
    }

    /**
     * Clear the current action. Note this does not revert an action that has been executed, but instead
     * allows new actions to trigger again. The current action is automatically cleared if all boolean suppliers
     * are returning false.
     */
    public void clearAction() {
        max = -1;
    }

    @Override
    public void run() {
        if (!hasResetWatchdogs) {
            watchdogs.forEach((d) -> {
                d.euthanize();
                d.start();
            });
            hasResetWatchdogs = true;
        }

        boolean hasTriggered = false;
        for (Pair<BooleanSupplier, TriggerAction> actionPair : actions) {
            if (actionPair.first.getAsBoolean()) {
                hasTriggered = true;
                int ordinal = actionPair.second.ordinal();
                if (ordinal > max) {
                    // Trigger actions are organised by "potency", so we'll run the highest action
                    max = ordinal;
                }
            }
        }

        if (!hasTriggered || max == -1) {
            clearAction();
            return;
        }

        require(opMode).getRobotControllers()
                .forEach((controller) -> controller.setConstant(Color.YELLOW));

        switch (TriggerAction.values()[max]) {
            case HALT:
                Dbg.warn(getClass(), "HALT safety triggered.");
                opMode.halt();
                break;
            case HALT_AND_STOP:
                Dbg.warn(getClass(), "HALT_AND_STOP safety triggered.");
                opMode.halt();
                opMode.stopMotors();
                break;
            case HALT_AND_POWER_DOWN:
                Dbg.warn(getClass(), "HALT_AND_POWER_DOWN safety triggered.");
                opMode.halt();
                opMode.safeHaltHardware();
                break;
            case STOP:
                Dbg.warn(getClass(), "STOP safety triggered.");
                opMode.exit();
                break;
            case TERMINATE:
                Dbg.warn(getClass(), "TERMINATE safety triggered.");
                opMode.emergencyStop();
                break;
        }
    }

    /**
     * Actions that can be executed on the event of a halting condition.
     */
    public enum TriggerAction {
        /**
         * Pause execution of the OpMode via {@code halt()}. Motors will
         * remain at their previous settings, not recommended.
         */
        HALT,
        /**
         * Pause execution of the OpMode via {@code halt()} and set all
         * motor/CRServo powers to 0 via {@code stopMotors()}.
         */
        HALT_AND_STOP,
        /**
         * Pause execution of the OpMode via {@code halt()} and safe shut
         * down all hardware via {@code safeHaltHardware()}.
         */
        HALT_AND_POWER_DOWN,
        /**
         * Stop execution of the OpMode via {@code exit()}, much as if the
         * user were to press the `STOP` button.
         */
        STOP,
        /**
         * Force-terminate the OpMode via method of a halting
         * exception ({@code emergencyStop()}).
         */
        TERMINATE
    }
}
