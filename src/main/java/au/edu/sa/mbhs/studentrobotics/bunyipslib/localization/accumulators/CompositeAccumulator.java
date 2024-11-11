package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Composes multiple {@link Accumulator} instances.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class CompositeAccumulator extends Accumulator {
    /**
     * A read-only list of all accumulators composed in this CompositeAccumulator.
     */
    public final List<Accumulator> accumulators;

    /**
     * Construct a composite accumulator from multiple accumulators.
     *
     * @param accumulators the accumulators to compose, will run each in order
     */
    public CompositeAccumulator(Accumulator... accumulators) {
        this(Arrays.asList(accumulators));
    }

    /**
     * Construct a composite accumulator from multiple accumulators.
     *
     * @param accumulators the accumulators to compose, will run each in order
     */
    public CompositeAccumulator(List<Accumulator> accumulators) {
        this.accumulators = Collections.unmodifiableList(accumulators);
    }

    @Override
    public void accumulate(@NonNull Twist2dDual<Time> twist) {
        for (Accumulator accumulator : accumulators) {
            accumulator.pose = pose;
            accumulator.velocity = velocity;
            accumulator.accumulate(twist);
            pose = accumulator.pose;
            velocity = accumulator.velocity;
        }
    }
}
