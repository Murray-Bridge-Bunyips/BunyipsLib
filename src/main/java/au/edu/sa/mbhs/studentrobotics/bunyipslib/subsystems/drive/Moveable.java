package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;

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
     * Set the current commanded power state of the robot in the Robot Coordinate System.
     *
     * @param target the normalised state this robot should now try to represent via motors running as a fraction
     *               of their maximum speed ([-1, 1])
     */
    void setPower(@NonNull PoseVelocity2d target);
}
