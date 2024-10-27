package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.List;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * Task to align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class AlignToContourTask extends Task {
    /**
     * PIDF coefficients for the alignment controller.
     */
    @NonNull
    public static PIDFCoefficients COEFFS = new PIDFCoefficients();

    private final Moveable drive;
    private final Supplier<List<ContourData>> contours;
    private final PIDF controller;
    private boolean hasCalculated;
    private Supplier<PoseVelocity2d> passthrough;

    /**
     * TeleOp constructor.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor   the vision processor to use
     * @param controller  the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF controller) {
        this(passthrough, drive, processor::getData, controller);
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver     the gamepad to use for driving
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF controller) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, processor::getData, controller);
    }

    /**
     * Autonomous constructor.
     *
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF controller) {
        this(drive, processor::getData, controller);
    }

    /**
     * TeleOp constructor.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier    a supplier source that will provide contour data
     * @param controller  the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF controller) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.controller = controller;
        this.passthrough = passthrough;
        controller.getPIDFController().updatePIDF(COEFFS);
        withName("Align To Contour");
        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver     the gamepad to use for driving
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier   a supplier source that will provide contour data
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF controller) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, supplier, controller);
    }

    /**
     * Autonomous constructor.
     *
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier   a supplier source that will provide contour data
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(@NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF controller) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.controller = controller;
        controller.getPIDFController().updatePIDF(COEFFS);
        withName("Align To Contour");
        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    @Override
    protected void init() {
        hasCalculated = false;
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.getPIDFController().setPIDF(COEFFS);

        PoseVelocity2d vel = Geometry.zeroVel();
        if (passthrough != null)
            vel = passthrough.get();

        List<ContourData> data = contours.get();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setPower(new PoseVelocity2d(
                    vel.linearVel,
                    controller.calculate(biggestContour.getYaw(), 0.0)
            ));
            hasCalculated = true;
        } else {
            drive.setPower(vel);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return passthrough == null && hasCalculated && controller.getPIDFController().atSetPoint();
    }
}
