package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * A function wrapper that only calls the function every so often at a user-defined interval.
 * <p>
 * This is useful for operations that should be delayed or slowed down, such as reading or writing to hardware.
 *
 * @author Lucas Bubner, 2024
 * @since 6.1.0
 */
public class Periodic implements Runnable {
    private final ElapsedTime timer = new ElapsedTime();
    private final Runnable function;
    private double intervalMs;
    private boolean init;

    /**
     * Creates a new periodic function wrapper.
     *
     * @param interval The interval at which to call the function. A zero or negative interval will execute the function on every call.
     * @param function The function to call.
     */
    public Periodic(Measure<Time> interval, Runnable function) {
        this.function = function;
        intervalMs = interval.in(Milliseconds);
    }

    /**
     * Sets the interval for this periodic function.
     * A zero or negative interval will execute the function on every call.
     *
     * @param interval the new interval
     */
    public void setInterval(@NonNull Measure<Time> interval) {
        intervalMs = interval.in(Milliseconds);
    }

    /**
     * Resets the timer for this periodic function.
     */
    public void reset() {
        init = false;
    }

    /**
     * Calls the function immediately, ignoring the set interval.
     */
    public void runNow() {
        function.run();
    }

    @Override
    public void run() {
        if (!init) {
            timer.reset();
            init = true;
        }

        if (intervalMs <= 0 || timer.milliseconds() >= intervalMs) {
            function.run();
            timer.reset();
        }
    }
}
