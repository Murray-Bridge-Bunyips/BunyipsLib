package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Interface for a generic object that may hold some current pose and velocity estimates.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public interface Localizable {
    /**
     * Calculate the accumulated pose from an internal localizer.
     *
     * @return the current pose estimate of the drive, may be nullable if this drive does not support localization
     */
    @Nullable
    Pose2d getPose();

    /**
     * Reset the accumulated pose estimate to this pose. Further updates from an internal localizer will continue
     * from this supplied pose estimate.
     *
     * @param newPose the new pose to continue localization from, may no-op if the drive does not support localization
     */
    void setPose(@NonNull Pose2d newPose);

    /**
     * Reset the accumulated pose estimate to this pose. Further updates from an internal localizer will continue
     * from this supplied pose estimate. This method internally composes a {@link Geometry} call to construct a united pose,
     * that will be interpreted back into an inches and radians pose.
     *
     * @param vector      the vector component of the pose
     * @param vectorUnit  the unit of the vector component distances
     * @param heading     the heading component of the pose
     * @param headingUnit the unit of the heading component angles
     */
    default void setPose(@NonNull Vector2d vector, @NonNull Distance vectorUnit, double heading, @NonNull Angle headingUnit) {
        setPose(Geometry.poseFrom(vector, vectorUnit, heading, headingUnit));
    }

    /**
     * Calculate the first derivative of the accumulated pose from an internal localizer.
     *
     * @return the current pose velocity of the drive, may be nullable if this drive does not support localization or
     * does not supply velocity
     */
    @Nullable
    PoseVelocity2d getVelocity();
}
