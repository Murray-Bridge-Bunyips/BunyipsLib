package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Inherited by the various field-centric compatible drive tasks to allow field-centric rotation and origin resets.
 *
 * @author Lucas Bubner, 2025
 * @see AlignToPointDriveTask
 * @see HolonomicDriveTask
 * @see HolonomicVectorDriveTask
 * @since 7.0.0
 */
public abstract class FieldOrientableDriveTask extends Task {
    protected Moveable drive; // must be assigned in inherited class

    protected Rotation2d fcOffset = Rotation2d.exp(0);
    protected BooleanSupplier fieldCentricEnabled = () -> false;

    protected PoseVelocity2d applyOrientation(PoseVelocity2d robotFrameVel) {
        if (fieldCentricEnabled.getAsBoolean()) {
            Pose2d currentPose = Objects.requireNonNull(drive.getPose(), "A heading localizer must be attached to the drive instance to allow for Field-Centric driving!");
            return currentPose.heading.inverse().times(fcOffset).times(robotFrameVel);
        }
        return robotFrameVel;
    }

    /**
     * Whether to use field-centric control on the passed-through translation.
     * <p>
     * This input rotation applies to suppliers that take in vector inputs as either passthrough or as the primary
     * drive vector input.
     *
     * @param enabled whether to use field-centric input control, rotating the input vector by the inverse current heading
     * @return this
     */
    @NonNull
    public FieldOrientableDriveTask withFieldCentric(@NonNull BooleanSupplier enabled) {
        fieldCentricEnabled = enabled;
        return this;
    }

    /**
     * Sets an angle to use as the origin for Field-Centric driving.
     * Only effective while field centric is active.
     *
     * @param fcOffset the offset angle (usually the current robot heading) to add to the vector heading rotation
     */
    public void setFieldCentricOffset(@NonNull Measure<Angle> fcOffset) {
        this.fcOffset = Rotation2d.exp(fcOffset.in(Radians));
    }

    /**
     * Sets the origin angle for Field-Centric driving to the current drive pose of the robot (effectively resetting the offset).
     * <p>
     * This is the most common use case for resetting the offset of FC operations.
     * Only effective while field centric is active.
     */
    public void resetFieldCentricOrigin() {
        fcOffset = Objects.requireNonNull(drive.getPose(), "Field-Centric cannot be used with no heading information").heading;
    }
}
