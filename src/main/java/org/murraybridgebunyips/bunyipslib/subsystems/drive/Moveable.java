package org.murraybridgebunyips.bunyipslib.subsystems.drive;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.murraybridgebunyips.bunyipslib.localization.Localizable;
import org.murraybridgebunyips.bunyipslib.localization.Localizer;

/**
 * A generic interface to represent drivetrains and other movement that may operate in the sense that the
 * current position can be modified on a 2D plane, and read back from accumulating a {@link Localizer}.
 * <p>
 * This interface does not make any assumptions on implementation details, or the hardware used, and simply
 * provides a way that odometry and drive motor propagation can link and be used in components and tasks.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public interface Moveable extends Localizable {
    /**
     * Set the current commanded state of the robot in the Robot Coordinate System.
     *
     * @param target the state this robot should now try to represent via motors
     */
    void setPower(PoseVelocity2d target);
}
