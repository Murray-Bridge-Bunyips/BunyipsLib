package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.Processor;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * Task to move to and align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class MoveToContourTask extends Task {
    /**
     * The PIDF coefficients for the translational controller.
     */
    @NonNull
    public static PIDFCoefficients TRANSLATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The PIDF coefficients for the rotational controller.
     */
    @NonNull
    public static PIDFCoefficients ROTATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The target position of the pixel on the camera pitch axis.
     */
    public static double PITCH_TARGET = 0.0;

    private final Moveable drive;
    private final Supplier<List<ContourData>> contours;
    private final PIDF translationController;
    private final PIDF rotationController;
    private boolean hasCalculated;
    private Supplier<PoseVelocity2d> passthrough;

    /**
     * TeleOp constructor.
     *
     * @param passthrough           the pose velocity passthrough for the drivetrain
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.passthrough = passthrough;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.getPIDFController().updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.getPIDFController().updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver                the gamepad to use for driving
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, supplier, translationController, rotationController);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Measure<Time> timeout, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        super(timeout);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.getPIDFController().updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.getPIDFController().updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    /**
     * TeleOp constructor.
     *
     * @param passthrough           the pose velocity passthrough for the drivetrain
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        this(passthrough, drive, processor::getData, translationController, rotationController);
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver                the gamepad to use for driving
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, processor::getData, translationController, rotationController);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(@NonNull Measure<Time> timeout, @NonNull Moveable drive, @NonNull Processor<ContourData> processor, @NonNull PIDF translationController, @NonNull PIDF rotationController) {
        this(timeout, drive, processor::getData, translationController, rotationController);
    }

    /**
     * Set the pitch target of where the pixel should be on the camera. 0.0 is the middle.
     *
     * @param pitchTarget the target pitch to move to
     * @return the task
     */
    @NonNull
    public MoveToContourTask withPitchTarget(double pitchTarget) {
        PITCH_TARGET = pitchTarget;
        return this;
    }

    @Override
    protected void init() {
        hasCalculated = false;
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        translationController.getPIDFController().setPIDF(TRANSLATIONAL_PIDF);
        rotationController.getPIDFController().setPIDF(ROTATIONAL_PIDF);

        PoseVelocity2d vel = Geometry.zeroVel();
        if (passthrough != null)
            vel = passthrough.get();

        List<ContourData> data = contours.get();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setPower(
                    Geometry.vel(
                            -translationController.calculate(biggestContour.getPitch(), Range.clip(PITCH_TARGET, -1.0, 1.0)),
                            vel.linearVel.y,
                            rotationController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
            hasCalculated = true;
        } else {
            drive.setPower(vel);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return passthrough == null && hasCalculated && translationController.getPIDFController().atSetPoint() && rotationController.getPIDFController().atSetPoint();
    }
}
