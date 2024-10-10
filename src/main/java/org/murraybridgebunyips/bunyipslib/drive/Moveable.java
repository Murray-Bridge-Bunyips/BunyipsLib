package org.murraybridgebunyips.bunyipslib.drive;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

/**
 * A generic interface to represent drivetrains and other movement that may operate in the sense that the
 * current position can be modified on a 2D plane, and read back from a {@link Localizer}.
 * <p>
 * This interface does not make any assumptions on implementation details, or the hardware used, and simply
 * provides a way that odometry and drive motor propagation can link and be used in components and tasks.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public interface Moveable {
    /**
     * Set the current commanded state of the robot in the Robot Coordinate System.
     *
     * @param directionalPower the state this robot should now try to represent via motors
     */
    void setPower(Pose2d directionalPower);

    /**
     * Get the currently used localizer attached to this drive instance.
     *
     * @return the localizer in use, may be null in drives that do not offer a localizer
     */
    @Nullable
    Localizer getLocalizer();
}
