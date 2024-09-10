package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Cartesian;
import org.murraybridgebunyips.bunyipslib.Controls;
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
 * @since 1.0.0-pre
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
     * @param xSupplier           The supplier for the Cartesian x-axis input
     * @param ySupplier           The supplier for the Cartesian y-axis input, <i>note that this will be inverted</i>
     * @param rSupplier           The supplier for the CW rotation input
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
        withName("Holonomic Control");
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
        double xV = x.get(), yV = y.get();
        if (drive instanceof MecanumDrive) {
            MecanumDrive d = (MecanumDrive) drive;
            Pose2d current = d.getPoseEstimate();
            if (fieldCentricEnabled.getAsBoolean()) {
                Vector2d cVec = Controls.makeCartesianVector(xV, yV);
                d.setWeightedDrivePower(new Pose2d(
                        Cartesian.toVector(Cartesian.rotate(cVec, Radians.of(current.getHeading()).negate())),
                        -r.get()
                ));
            } else {
                d.setSpeedUsingController(xV, yV, r.get());
            }
        } else if (drive instanceof CartesianMecanumDrive) {
            ((CartesianMecanumDrive) drive).setSpeedUsingController(xV, yV, r.get());
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
