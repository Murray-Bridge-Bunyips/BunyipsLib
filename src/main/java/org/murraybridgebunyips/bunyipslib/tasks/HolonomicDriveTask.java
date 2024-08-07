package org.murraybridgebunyips.bunyipslib.tasks;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @author Lucas Bubner, 2024
 */
public class HolonomicDriveTask extends ForeverTask {
    private final BunyipsSubsystem drive;
    private final Supplier<Float> x;
    private final Supplier<Float> y;
    private final Supplier<Float> r;
    private final BooleanSupplier fieldCentricEnabled;

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param xSupplier           The supplier for the x-axis input
     * @param ySupplier           The supplier for the y-axis input, <i>note that this will be inverted</i>
     * @param rSupplier           The supplier for the rotation input
     * @param mecanumDrive        The MecanumDrive to use for driving, must be a MecanumDrive or CartesianMecanumDrive
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Supplier<Float> xSupplier, Supplier<Float> ySupplier, Supplier<Float> rSupplier, @NotNull BunyipsSubsystem mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        if (!(mecanumDrive instanceof MecanumDrive) && !(mecanumDrive instanceof CartesianMecanumDrive))
            throw new EmergencyStop("HolonomicDriveTask must be used with a holonomic drivetrain");
        onSubsystem(mecanumDrive, false);
        drive = mecanumDrive;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;
        withName("Holonomic Drive Control");
    }

    /**
     * Constructor for HolonomicDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver              The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use for driving, must be a MecanumDrive or CartesianMecanumDrive
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Gamepad driver, @NotNull BunyipsSubsystem mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, mecanumDrive, fieldCentricEnabled);
    }

    @Override
    protected void periodic() {
        if (drive instanceof MecanumDrive) {
            if (fieldCentricEnabled.getAsBoolean()) {
                ((MecanumDrive) drive).setSpeedUsingControllerFieldCentric(x.get(), y.get(), r.get());
            } else {
                ((MecanumDrive) drive).setSpeedUsingController(x.get(), y.get(), r.get());
            }
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).setSpeedUsingController(x.get(), y.get(), r.get());
        }
    }

    @Override
    protected void onFinish() {
        if (drive instanceof MecanumDrive) {
            ((MecanumDrive) drive).stop();
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).stop();
        }
    }
}
