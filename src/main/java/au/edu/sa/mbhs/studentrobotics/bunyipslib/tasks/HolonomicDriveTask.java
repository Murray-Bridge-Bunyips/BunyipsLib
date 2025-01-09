package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class HolonomicDriveTask extends FieldOrientableDriveTask {
    private final Supplier<PoseVelocity2d> vel;

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param velSupplier The supplier for Robot velocity input
     * @param drive       The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                    called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                    if possible.
     */
    public HolonomicDriveTask(@NonNull Supplier<PoseVelocity2d> velSupplier, @NonNull Moveable drive) {
        super.drive = drive;
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        vel = velSupplier;
        named("Holonomic Control");
    }

    /**
     * Constructor for HolonomicDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver The gamepad to use for driving in a standard configuration
     * @param drive  The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *               called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *               if possible.
     */
    public HolonomicDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive);
    }

    @Override
    protected void periodic() {
        drive.setPower(applyOrientation(vel.get()));
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}
