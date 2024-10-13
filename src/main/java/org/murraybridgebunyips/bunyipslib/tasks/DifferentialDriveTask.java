package org.murraybridgebunyips.bunyipslib.tasks;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Geometry;
import org.murraybridgebunyips.bunyipslib.subsystems.drive.Moveable;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * Standard gamepad drive for all differential drivetrains.
 * Left stick Y controls forward translation, right stick controls rotation.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class DifferentialDriveTask extends ForeverTask {
    private final Moveable drive;
    private final Gamepad gamepad;

    /**
     * Constructs a new DifferentialDriveTask.
     *
     * @param gamepad the gamepad to use
     * @param drive   the drive to control, this task may be attached on this BunyipsSubsystem if applicable
     */
    public DifferentialDriveTask(Gamepad gamepad, @NotNull Moveable drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        withName("Differential Control");
    }

    @Override
    protected void periodic() {
        drive.setPower(Controls.makeRobotVel(gamepad.left_stick_y, 0, gamepad.right_stick_x));
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}
