package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Cartesian;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Geometry;
import org.murraybridgebunyips.bunyipslib.localization.Localizer;
import org.murraybridgebunyips.bunyipslib.subsystems.drive.Moveable;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.Objects;
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
    private final Moveable drive;
    private final Supplier<Float> x;
    private final Supplier<Float> y;
    private final Supplier<Float> r;
    private final BooleanSupplier fieldCentricEnabled;

    /**
     * Constructor for HolonomicDriveTask.
     *
     * @param xSupplier           The supplier for the Cartesian x-axis input
     * @param yInvSupplier        The supplier for the Cartesian y-axis input, <i>note that this will be inverted</i>
     * @param rSupplier           The supplier for the CW rotation input
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled,
     *                            note this will only work on a drive that has a {@link Localizer} attached to it.
     */
    public HolonomicDriveTask(Supplier<Float> xSupplier, Supplier<Float> yInvSupplier, Supplier<Float> rSupplier, @NotNull Moveable drive, BooleanSupplier fieldCentricEnabled) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        x = xSupplier;
        y = yInvSupplier;
        r = rSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;
        withName("Holonomic Control");
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
    public HolonomicDriveTask(Gamepad driver, @NotNull Moveable drive, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, fieldCentricEnabled);
    }

    @Override
    protected void periodic() {
        double xV = x.get(), yV = y.get();
        if (fieldCentricEnabled.getAsBoolean()) {
            Pose2d currentPose = Objects.requireNonNull(drive.getPoseEstimate(), "A heading localizer must be attached to the drive instance to allow for Field-Centric driving!");
            Vector2d cVec = Controls.makeCartesianVector(xV, yV);
            drive.setPower(Geometry.poseToVel(new Pose2d(
                    Cartesian.toVector(Cartesian.rotate(cVec, Radians.of(-currentPose.heading.toDouble()))),
                    -r.get())
            ));
            return;
        }
        drive.setPower(Geometry.poseToVel(Controls.makeRobotPose(xV, yV, r.get())));
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}
