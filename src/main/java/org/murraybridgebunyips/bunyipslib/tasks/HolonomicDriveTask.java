package org.murraybridgebunyips.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.Driver;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Standard gamepad drive for all holonomic drivetrains.
 * Left stick controls translation, right stick controls rotation.
 * This task is designed to be used as a default task, other tasks will override it.
 *
 * @param <T> The type of the MecanumDrive to use
 * @author Lucas Bubner, 2024
 */
public class HolonomicDriveTask<T extends BunyipsSubsystem> extends ForeverTask {
    private final T drive;
    private final Driver controller;
    private final BooleanSupplier fieldCentricEnabled;
    private final DoubleSupplier multiplierSupplier;

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param controller             The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use for driving
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Driver controller, @NonNull T mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        this(controller, mecanumDrive, () -> 1.0, fieldCentricEnabled);
    }

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param controller             The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use for driving
     * @param multiplierSupplier  Supply a multiplier that will be applied to the inputs of each joystick.
     *                            Useful when in use with the InputMultiplier.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            this will only work on a MecanumDrive that supports dynamic field-centric
     *                            drive switching, such as the RoadRunner-integrated MecanumDrive
     */
    public HolonomicDriveTask(Driver controller, @NotNull T mecanumDrive, DoubleSupplier multiplierSupplier, BooleanSupplier fieldCentricEnabled) {
        super(mecanumDrive, false);
        if (!(mecanumDrive instanceof MecanumDrive) && !(mecanumDrive instanceof CartesianMecanumDrive))
            throw new EmergencyStop("HolonomicDriveTask must be used with a holonomic drivetrain");
        drive = mecanumDrive;
        this.controller = controller;
        this.multiplierSupplier = multiplierSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;
    }

    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        double x = controller.lx();
        double y = controller.ly();
        double r = controller.rx();

        if (drive instanceof MecanumDrive) {
            if (fieldCentricEnabled.getAsBoolean()) {
                ((MecanumDrive) drive).setSpeedUsingControllerFieldCentric(
                        x * multiplierSupplier.getAsDouble(),
                        y * multiplierSupplier.getAsDouble(),
                        r * multiplierSupplier.getAsDouble()
                );
            } else {
                ((MecanumDrive) drive).setSpeedUsingController(
                        x * multiplierSupplier.getAsDouble(),
                        y * multiplierSupplier.getAsDouble(),
                        r * multiplierSupplier.getAsDouble()
                );
            }
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).setSpeedUsingController(
                    x * multiplierSupplier.getAsDouble(),
                    y * multiplierSupplier.getAsDouble(),
                    r * multiplierSupplier.getAsDouble()
            );
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
