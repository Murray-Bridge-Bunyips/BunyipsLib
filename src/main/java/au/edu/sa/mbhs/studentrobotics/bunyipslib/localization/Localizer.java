package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

/**
 * Interface for a generic localizer that may track the movement deltas of a robot over time.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@FunctionalInterface
public interface Localizer {
    /**
     * Perform one iteration of localizer delta updates.
     *
     * @return the twist delta of pose since the last update, units of inches and radians
     */
    @NonNull
    Twist2dDual<Time> update();
}
