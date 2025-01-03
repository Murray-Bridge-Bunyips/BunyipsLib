package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class HolonomicDriveTask extends Task {
    private final Moveable drive;
    private final Supplier<PoseVelocity2d> vel;
    private final BooleanSupplier fieldCentricEnabled;
    private Rotation2d fcOffset = Rotation2d.exp(0);

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param velSupplier         The supplier for Robot velocity input
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            note this will only work on a drive that has a {@link Localizer} attached to it.
     */
    public HolonomicDriveTask(@NonNull Supplier<PoseVelocity2d> velSupplier, @NonNull Moveable drive, @NonNull BooleanSupplier fieldCentricEnabled) {
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        this.drive = drive;
        vel = velSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;
        named("Holonomic Control");
    }

    /**
     * Constructor for HolonomicDriveTask on an always disabled field-centric mode.
     *
     * @param velSupplier The supplier for Robot velocity input
     * @param drive       The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                    called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                    if possible.
     */
    public HolonomicDriveTask(@NonNull Supplier<PoseVelocity2d> velSupplier, @NonNull Moveable drive) {
        this(velSupplier, drive, () -> false);
    }

    /**
     * Constructor for HolonomicDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver              The gamepad to use for driving in a standard configuration
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            note this will only work on a drive that has a {@link Localizer} attached to it.
     */
    public HolonomicDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull BooleanSupplier fieldCentricEnabled) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, fieldCentricEnabled);
    }

    /**
     * Constructor for HolonomicDriveTask using a default Mecanum binding. Field-centric mode is disabled by default.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver The gamepad to use for driving in a standard configuration
     * @param drive  The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *               called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *               if possible.
     */
    public HolonomicDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive) {
        this(driver, drive, () -> false);
    }

    /**
     * Sets an angle to use as the origin for Field-Centric driving.
     * If this mode is not enabled on the drive task, this value won't be used for anything meaningful.
     *
     * @param fcOffset the offset angle (usually the current robot heading) to add to the vector heading rotation
     */
    public void setFieldCentricOffset(@NonNull Measure<Angle> fcOffset) {
        this.fcOffset = Rotation2d.exp(fcOffset.in(Radians));
    }

    /**
     * Sets the origin angle for Field-Centric driving to the drive pose of the robot (effectively resetting the offset).
     * This is the most common use case for resetting the offset of FC operations.
     * If this mode is not enabled on the drive task, this value won't be used for anything meaningful.
     *
     * @param drivePose the current pose of the drive that will be used to zero out the field centric origin
     */
    public void resetFieldCentricOrigin(@NonNull Pose2d drivePose) {
        fcOffset = drivePose.heading;
    }

    @Override
    protected void periodic() {
        if (fieldCentricEnabled.getAsBoolean()) {
            Pose2d currentPose = Objects.requireNonNull(drive.getPose(), "A heading localizer must be attached to the drive instance to allow for Field-Centric driving!");
            drive.setPower(currentPose.heading.inverse().times(fcOffset).times(vel.get()));
            return;
        }
        drive.setPower(vel.get());
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}
