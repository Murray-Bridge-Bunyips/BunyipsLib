package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.drive.Moveable;
import org.murraybridgebunyips.bunyipslib.external.PIDF;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Task to align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
@Config
public class AlignToContourTask extends Task {
    /**
     * PIDF coefficients for the alignment controller.
     */
    public static PIDFCoefficients coeffs = new PIDFCoefficients();

    private final Moveable drive;
    private final Supplier<List<ContourData>> contours;
    private final PIDF controller;
    private boolean hasCalculated;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    /**
     * TeleOp constructor
     *
     * @param xSupplier  x (strafe) value
     * @param ySupplier  y (forward) value
     * @param rSupplier  r (rotate) value
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, Moveable drive, Processor<ContourData> processor, PIDF controller) {
        this(xSupplier, ySupplier, rSupplier, drive, processor::getData, controller);
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver     the gamepad to use for driving
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Gamepad driver, Moveable drive, Processor<ContourData> processor, PIDF controller) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, processor::getData, controller);
    }

    /**
     * Autonomous constructor
     *
     * @param timeout    the maximum time in seconds to run the task for
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Measure<Time> timeout, Moveable drive, Processor<ContourData> processor, PIDF controller) {
        this(timeout, drive, processor::getData, controller);
    }

    /**
     * TeleOp constructor.
     *
     * @param xSupplier  x (strafe) value
     * @param ySupplier  y (forward) value
     * @param rSupplier  r (rotate) value
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier   a supplier source that will provide contour data
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, Moveable drive, Supplier<List<ContourData>> supplier, PIDF controller) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.controller = controller;
        controller.getPIDFController().updatePIDF(coeffs);
        withName("Align To Contour");
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver     the gamepad to use for driving
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier   a supplier source that will provide contour data
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Gamepad driver, Moveable drive, Supplier<List<ContourData>> supplier, PIDF controller) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, supplier, controller);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout    the maximum time in seconds to run the task for
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier   a supplier source that will provide contour data
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Measure<Time> timeout, Moveable drive, Supplier<List<ContourData>> supplier, PIDF controller) {
        super(timeout);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.controller = controller;
        controller.getPIDFController().updatePIDF(coeffs);
        withName("Align To Contour");
    }

    @Override
    protected void init() {
        hasCalculated = false;
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.getPIDFController().setPIDF(coeffs);

        Pose2d pose = new Pose2d(0, 0, 0);
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<ContourData> data = contours.get();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setPower(
                    new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            controller.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
            hasCalculated = true;
        } else {
            drive.setPower(pose);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return x == null && hasCalculated && controller.getPIDFController().atSetPoint();
    }
}
