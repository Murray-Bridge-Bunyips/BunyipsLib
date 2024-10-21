package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Collection of {@link BooleanSupplier} extensions for rising and falling edge detection.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class Condition implements BooleanSupplier {
    private final BooleanSupplier condition;
    /**
     * The currently set edge detection to use for the {@link BooleanSupplier} {@link #getAsBoolean()} method.
     */
    public Edge edge;
    private long delayNs;
    private long capture;
    private boolean lastState;

    /**
     * Creates a new Condition with the given {@link BooleanSupplier}.
     *
     * @param edge      The edge detection to use.
     * @param condition The {@link BooleanSupplier} to use.
     */
    public Condition(Edge edge, BooleanSupplier condition) {
        this.condition = condition;
        this.edge = edge;
    }

    /**
     * Creates a new Condition with the given {@link BooleanSupplier} and no edge detection.
     *
     * @param condition The {@link BooleanSupplier} to use.
     */
    public Condition(BooleanSupplier condition) {
        this(Edge.ACTIVE, condition);
    }

    /**
     * Composes this Condition with a delay. The delay will be applied to the output of all exposed methods of this
     * class and dictates a minimum time that the original condition must be true for the output to be true.
     * <p>
     * This is useful for ensuring that a condition is true for a certain amount of time before acting on it.
     * <p>
     * On rising/falling edge detection, the delay will be applied to the raw condition itself, before
     * edge detection is performed and outputted through the accessors.
     *
     * @param delay the delay to apply to the condition. Zero or negative values will disable the delay (default).
     * @return this Condition with the delay applied
     */
    public Condition withActiveDelay(@Nullable Measure<Time> delay) {
        if (delay == null) {
            delayNs = 0;
            return this;
        }
        double d = delay.in(Nanoseconds);
        delayNs = d <= 0 ? 0 : (long) d;
        return this;
    }

    /**
     * @return the currently set delay for the condition.
     */
    public Measure<Time> getActiveDelay() {
        return Nanoseconds.of(delayNs);
    }

    /**
     * @return the value of the condition as dictated by the current {@link Edge} setting.
     */
    @Override
    public boolean getAsBoolean() {
        switch (edge) {
            case RISING:
                return getRisingEdge();
            case FALLING:
                return getFallingEdge();
            case ACTIVE:
            default:
                return getActive();
        }
    }

    private boolean timed(boolean currentState) {
        if (delayNs <= 0)
            return currentState;
        if (currentState) {
            if (capture == 0) {
                capture = System.nanoTime();
            } else {
                return System.nanoTime() - capture >= delayNs;
            }
        } else {
            capture = 0;
        }
        return false;
    }

    /**
     * @return {@code true} if the condition is true, {@code false} otherwise.
     */
    public boolean getActive() {
        return timed(condition.getAsBoolean());
    }

    /**
     * @return {@code true} if the condition was last false, {@code false} otherwise.
     */
    public boolean getRisingEdge() {
        boolean currentState = timed(condition.getAsBoolean());
        if (currentState && !lastState) {
            lastState = true;
            return true;
        } else if (!currentState) {
            lastState = false;
        }
        return false;
    }

    /**
     * @return {@code true} if the condition was last true, {@code false} otherwise.
     */
    public boolean getFallingEdge() {
        boolean currentState = timed(condition.getAsBoolean());
        if (!currentState && lastState) {
            lastState = false;
            return true;
        } else if (currentState) {
            lastState = true;
        }
        return false;
    }

    /**
     * Creates a new Condition that is the logical OR of this Condition and another @link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to OR with.
     * @return A new Condition that is the logical OR of this Condition and the other {@link BooleanSupplier}.
     */
    public Condition or(BooleanSupplier other) {
        return new Condition(edge, () -> getAsBoolean() || other.getAsBoolean());
    }

    /**
     * Creates a new Condition that is the logical AND of this Condition and anot @link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to AND with.
     * @return A new Condition that is the logical AND of this Condition and the other {@link BooleanSupplier}.
     */
    public Condition and(BooleanSupplier other) {
        return new Condition(edge, () -> getAsBoolean() && other.getAsBoolean());
    }

    /**
     * Creates a new Condition that is the logical XOR of this Condition and anot @link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to XOR with.
     * @return A new Condition that is the logical XOR of this Condition and the other {@link BooleanSupplier}.
     */
    public Condition xor(BooleanSupplier other) {
        return new Condition(edge, () -> getAsBoolean() ^ other.getAsBoolean());
    }

    /**
     * Creates a new Condition that is the logical NOT of this Condition.
     *
     * @return A new Condition that is the logical NOT of this Condition.
     */
    public Condition not() {
        return new Condition(edge, () -> !getAsBoolean());
    }

    @NonNull
    @Override
    public String toString() {
        return "Condition{" +
                "condition=" + condition +
                ", edge=" + edge +
                ", delayMs=" + delayNs / 1.0e6 +
                '}';
    }

    /**
     * Enum for edge detection.
     */
    public enum Edge {
        /**
         * Perform no edge detection (turns this class into a normal {@link BooleanSupplier}).
         */
        ACTIVE,
        /**
         * Perform rising edge detection such that the condition is true only when it was false in the last cycle.
         */
        RISING,
        /**
         * Perform falling edge detection such that the condition is true only when it was true in the last cycle.
         */
        FALLING
    }
}
