package org.murraybridgebunyips.bunyipslib.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * Interface for a generic object that may hold some current pose estimate and velocity.
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
    Pose2d getPoseEstimate();

    /**
     * Reset the accumulated pose estimate to this pose. Further updates from an internal localizer will continue
     * from this supplied pose estimate.
     *
     * @param newPose the new pose to continue localization from, may no-op if the drive does not support localization
     */
    void setPoseEstimate(@NonNull Pose2d newPose);

    /**
     * Calculate the first derivative of the accumulated pose from an internal localizer.
     *
     * @return the current pose velocity of the drive, may be nullable if this drive does not support localization or
     * does not supply velocity
     */
    @Nullable
    PoseVelocity2d getPoseVelocity();
}
