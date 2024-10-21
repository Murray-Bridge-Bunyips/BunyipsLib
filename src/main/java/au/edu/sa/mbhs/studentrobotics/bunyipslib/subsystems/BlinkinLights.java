package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.IdleTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.RunForTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

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
    public BlinkinLights(@NonNull RevBlinkinLedDriver lights, @NonNull RevBlinkinLedDriver.BlinkinPattern defaultPattern) {
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
    @NonNull
    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return currentPattern;
    }

    /**
     * Set the current pattern. Will no-op if a task is running on this subsystem.
     *
     * @param pattern the pattern to update to.
     * @return this
     */
    @NonNull
    public BlinkinLights setPattern(@NonNull RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (getCurrentTask() instanceof IdleTask)
            currentPattern = pattern;
        return this;
    }

    /**
     * Get the current default pattern.
     *
     * @return currently respected default pattern.
     */
    @NonNull
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
    @NonNull
    public BlinkinLights setDefaultPattern(@NonNull RevBlinkinLedDriver.BlinkinPattern defaultPattern) {
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
    @NonNull
    public BlinkinLights forceSetPattern(@NonNull RevBlinkinLedDriver.BlinkinPattern pattern) {
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
    @NonNull
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
    @NonNull
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
        opMode(o -> o.telemetry.add("%: Pattern->%", this, currentPattern.name()).color("gray"));
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
        @NonNull
        public Task setPatternFor(@NonNull Measure<Time> duration, @NonNull RevBlinkinLedDriver.BlinkinPattern pattern) {
            return new RunForTask(duration, () -> currentPattern = pattern, () -> currentPattern = defaultPattern)
                    .onSubsystem(BlinkinLights.this, true)
                    .withName("Lights:" + pattern.name());
        }
    }
}
