package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Ref;
import dev.frozenmilk.util.cell.RefCell;

/**
 * Interface for a generic localizer that may track the movement deltas of a robot over time.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@FunctionalInterface
public interface Localizer {
    /**
     * The interval at which to auto-log localizer inputs to the {@link FlightRecorder}.
     * Requires a reinitialisation if changed.
     */
    RefCell<Long> FLIGHT_RECORDER_INTERVAL_MS = Ref.of(25L);

    /**
     * Perform one iteration of localizer delta updates.
     *
     * @return the twist delta of pose since the last update, units of inches and radians
     */
    @NonNull
    Twist2dDual<Time> update();
}
