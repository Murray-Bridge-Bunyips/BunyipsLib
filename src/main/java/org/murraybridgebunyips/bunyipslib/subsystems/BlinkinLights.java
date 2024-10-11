package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.IdleTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunForTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * LED driver subsystem for the REV Blinkin Lights. Able to integrate with the Task system to schedule lighting patterns.
 *
 * @author Lachlan Paul, 2024
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class BlinkinLights extends BunyipsSubsystem {
    /**
     * Tasks for BlinkinLights.
     */
    public final Tasks tasks = new Tasks();

    private final RevBlinkinLedDriver lights;
    private RevBlinkinLedDriver.BlinkinPattern defaultPattern;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;
    private RevBlinkinLedDriver.BlinkinPattern setPattern;

    /**
     * Create a new BlinkinLights subsystem.
     *
     * @param lights         the LED driver to use.
     * @param defaultPattern the default pattern which will be the one this driver goes back to as a default.
     */
    public BlinkinLights(RevBlinkinLedDriver lights, RevBlinkinLedDriver.BlinkinPattern defaultPattern) {
        this.lights = lights;
        this.defaultPattern = defaultPattern;
        currentPattern = defaultPattern;

        if (!assertParamsNotNull(lights)) return;
        lights.setPattern(this.defaultPattern);
    }

    /**
     * Get the current pattern.
     *
     * @return currently respected pattern
     */
    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return currentPattern;
    }

    /**
     * Set the current pattern. Will no-op if a task is running on this subsystem.
     *
     * @param pattern the pattern to update to.
     * @return this
     */
    public BlinkinLights setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (getCurrentTask() instanceof IdleTask)
            currentPattern = pattern;
        return this;
    }

    /**
     * Get the current default pattern.
     *
     * @return currently respected default pattern.
     */
    public RevBlinkinLedDriver.BlinkinPattern getDefaultPattern() {
        return defaultPattern;
    }

    /**
     * Set the default pattern. The default pattern is the one this driver uses as a default, and is specified
     * previously in the constructor. This method will update the current pattern to the new default if the old
     * default is the current pattern.
     *
     * @param defaultPattern the new default pattern
     * @return this
     */
    public BlinkinLights setDefaultPattern(RevBlinkinLedDriver.BlinkinPattern defaultPattern) {
        if (this.defaultPattern == currentPattern)
            currentPattern = defaultPattern;
        this.defaultPattern = defaultPattern;
        return this;
    }

    /**
     * Set the current pattern. Will internally cancel any running task on this subsystem.
     *
     * @param pattern the pattern to update to.
     * @return this
     */
    public BlinkinLights forceSetPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (!(getCurrentTask() instanceof IdleTask))
            cancelCurrentTask();
        currentPattern = pattern;
        return this;
    }

    /**
     * Reset the pattern back to the default. Will no-op if a task is running on this subsystem.
     *
     * @return this
     */
    public BlinkinLights resetPattern() {
        if (getCurrentTask() instanceof IdleTask)
            currentPattern = defaultPattern;
        return this;
    }

    /**
     * Cancel tasks and turn off the lights. Auto-called on subsystem disable.
     *
     * @return this
     */
    public BlinkinLights turnOff() {
        cancelCurrentTask();
        currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        return this;
    }

    @Override
    protected void onDisable() {
        turnOff();
    }

    @Override
    protected void onEnable() {
        currentPattern = defaultPattern;
    }

    @Override
    protected void periodic() {
        opMode(o -> o.telemetry.add("%: Pattern->%", name, currentPattern.name()).color("gray"));
        if (setPattern != currentPattern) {
            lights.setPattern(currentPattern);
            setPattern = currentPattern;
        }
    }

    /**
     * Tasks for BlinkinLights, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Set the current pattern for a set duration.
         *
         * @param duration the time duration
         * @param pattern  the pattern to set
         * @return a task to set the pattern for a duration
         */
        public Task setPatternFor(Measure<Time> duration, RevBlinkinLedDriver.BlinkinPattern pattern) {
            return new RunForTask(duration, () -> currentPattern = pattern, () -> currentPattern = defaultPattern)
                    .onSubsystem(BlinkinLights.this, true)
                    .withName("Lights:" + pattern.name());
        }
    }
}
