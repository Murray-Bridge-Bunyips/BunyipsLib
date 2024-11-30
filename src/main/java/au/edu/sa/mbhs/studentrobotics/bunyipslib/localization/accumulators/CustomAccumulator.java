package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * An accumulator that is defined by the user.
 * <p>
 * Custom accumulators can be used to combine accumulator readings, or to provide custom logic for the accumulation of
 * pose estimates.
 *
 * @author Lucas Bubner, 2024
 * @see Accumulator
 * @since 6.0.0
 */
public abstract class CustomAccumulator extends Accumulator {
    /**
     * A read-only list of all accumulators registered in this CustomAccumulator.
     */
    @NonNull
    public List<Accumulator> registeredAccumulators = Collections.unmodifiableList(new ArrayList<>());

    /**
     * Register accumulators that are used internally by this custom accumulator.
     *
     * @param accumulators the accumulators to register
     */
    public void register(@NonNull Accumulator... accumulators) {
        registeredAccumulators = List.of(accumulators);
    }

    /**
     * Run one accumulation of this accumulator to update the pose with this delta.
     * <p>
     * Call {@link #internalAccumulate(Twist2dDual)} to update the pose and velocity with the twist as per a standard
     * accumulator, which also updates FtcDashboard and the pose history.
     *
     * @param twist the change in position and velocity with respect to time
     */
    public abstract void accumulate(@NonNull Twist2dDual<Time> twist);

    /**
     * Run one accumulation of the underlying accumulator to update the pose with this delta.
     * The pose and pose history on FtcDashboard will also be updated on this accumulation.
     *
     * @param twist the change in position and velocity with respect to time
     */
    public void internalAccumulate(@NonNull Twist2dDual<Time> twist) {
        super.accumulate(twist);
    }
}
