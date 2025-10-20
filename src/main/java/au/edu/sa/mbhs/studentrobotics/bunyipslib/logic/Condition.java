package au.edu.sa.mbhs.studentrobotics.bunyipslib.logic;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Collection of {@link BooleanSupplier} extensions for rising and falling edge detection.
 * <p>
 * Supports self-typing for type-safe chaining of boolean conditions.
 *
 * @param <T> self-type
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@SuppressWarnings("unchecked")
public class Condition<T extends Condition<T>> implements BooleanSupplier {
    private final BooleanSupplier condition;
    /**
     * The currently set edge detection to use for the {@link BooleanSupplier} {@link #getAsBoolean()} method.
     */
    @NonNull
    public Edge edge;
    private long delayNs;
    private long capture;
    private boolean lastState;

    /**
     * Creates a new self-type Condition with the given {@link BooleanSupplier}.
     *
     * @param edge      The edge detection to use.
     * @param condition The {@link BooleanSupplier} to use.
     */
    public Condition(@NonNull Edge edge, @NonNull BooleanSupplier condition) {
        this.edge = edge;
        if (condition instanceof Condition<?> self) {
            // Flatten new Condition instances
            this.condition = self.condition;
            self.delayNs = delayNs;
            self.capture = capture;
            self.lastState = lastState;
        } else {
            this.condition = condition;
        }
    }

    /**
     * Creates a new self-type Condition with the given {@link BooleanSupplier} and no edge detection.
     *
     * @param condition The {@link BooleanSupplier} to use.
     */
    public Condition(@NonNull BooleanSupplier condition) {
        this(Edge.ACTIVE, condition);
    }

    /**
     * Mutates this Condition to adjust the edge detection for
     * the {@link BooleanSupplier} {@link #getAsBoolean()} method.
     *
     * @param edge the edge to use
     * @return self with the edge applied
     */
    @NonNull
    public T withEdge(@NonNull Edge edge) {
        this.edge = edge;
        return (T) this;
    }

    /**
     * Mutates this Condition with a delay. The delay will be applied to the output of all exposed methods of this
     * class and dictates a minimum time that the original condition must be true for the output to be true.
     * <p>
     * This is useful for ensuring that a condition is true for a certain amount of time before acting on it.
     * <p>
     * On rising/falling edge detection, the delay will be applied to the edge condition, such that
     * this supplier will return true for one iteration after delay has passed for rising, and for falling
     * will fire after the delay has passed after the condition has transitioned from true to false and has remained false.
     *
     * @param delay the delay to apply to the condition. Zero or negative values will disable the delay (default).
     * @return self with the delay applied
     */
    @NonNull
    public T withActiveDelay(@Nullable Measure<Time> delay) {
        if (delay == null) {
            delayNs = 0;
            return (T) this;
        }
        double d = delay.in(Nanoseconds);
        delayNs = d <= 0 ? 0 : (long) d;
        return (T) this;
    }

    /**
     * @return the currently set delay for the condition.
     */
    @NonNull
    public Measure<Time> getActiveDelay() {
        return Nanoseconds.of(delayNs);
    }

    /**
     * @return the value of the condition as dictated by the current {@link Edge} setting.
     */
    @Override
    public boolean getAsBoolean() {
        return switch (edge) {
            case RISING -> getRisingEdge();
            case FALLING -> getFallingEdge();
            default -> getActive();
        };
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
        boolean currentState = condition.getAsBoolean();
        if (!currentState && timed(lastState)) {
            lastState = false;
            return true;
        } else if (currentState) {
            capture = System.nanoTime();
            lastState = true;
        }
        return false;
    }

    /**
     * Override this method and construct a new instance of the subclass for self-typing.
     *
     * @param edge     supplied edge for construction
     * @param supplier supplied supplier for construction
     * @return new instance of type T
     */
    protected T newInstance(Condition.Edge edge, BooleanSupplier supplier) {
        return (T) new Condition<>(edge, supplier);
    }

    /**
     * Creates a new self-type Condition that is the logical OR of this Condition and another {@link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to OR with.
     * @return A new self-type Condition that is the logical OR of this Condition and the other {@link BooleanSupplier}.
     */
    @NonNull
    public T or(@NonNull BooleanSupplier other) {
        return newInstance(edge, new Or(this, other));
    }

    /**
     * Creates a new self-type Condition that is the logical AND of this Condition and another {@link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to AND with.
     * @return A new self-type Condition that is the logical AND of this Condition and the other {@link BooleanSupplier}.
     */
    @NonNull
    public T and(@NonNull BooleanSupplier other) {
        return newInstance(edge, new And(this, other));
    }

    /**
     * Creates a new self-type Condition that is the logical XOR of this Condition and another {@link BooleanSupplier}.
     *
     * @param other The other {@link BooleanSupplier} to XOR with.
     * @return A new self-type Condition that is the logical XOR of this Condition and the other {@link BooleanSupplier}.
     */
    @NonNull
    public T xor(@NonNull BooleanSupplier other) {
        return newInstance(edge, new Xor(this, other));
    }

    /**
     * Creates a new self-type Condition that is the logical NOT of this Condition.
     *
     * @return A new self-type Condition that is the logical NOT of this Condition.
     */
    @NonNull
    public T not() {
        return newInstance(edge, new Not(this));
    }

    @NonNull
    @Override
    public String toString() {
        if (edge != Edge.ACTIVE || delayNs != 0)
            return condition + "{" + edge + "," + Mathf.round(delayNs / 1.0e6, 1) + "ms}";
        return condition.toString();
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
