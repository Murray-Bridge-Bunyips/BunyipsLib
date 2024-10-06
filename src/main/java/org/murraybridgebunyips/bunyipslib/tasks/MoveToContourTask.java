package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.external.PIDF;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Task to move to and align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
@Config
public class MoveToContourTask extends Task {
    /**
     * The PIDF coefficients for the translational controller.
     */
    public static PIDFCoefficients TRANSLATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The PIDF coefficients for the rotational controller.
     */
    public static PIDFCoefficients ROTATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The target position of the pixel on the camera pitch axis.
     */
    public static double PITCH_TARGET = 0.0;

    private final RoadRunnerDrive drive;
    private final Supplier<List<ContourData>> contours;
    private final PIDF translationController;
    private final PIDF rotationController;
    private boolean hasCalculated;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    /**
     * TeleOp constructor.
     *
     * @param xSupplier             x (strafe) value
     * @param ySupplier             y (forward) value
     * @param rSupplier             r (rotate) value
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, BunyipsSubsystem drive, Supplier<List<ContourData>> supplier, PIDF translationController, PIDF rotationController) {
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        onSubsystem(drive, false);
        this.drive = (RoadRunnerDrive) drive;
        contours = supplier;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.getPIDFController().updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.getPIDFController().updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver                the gamepad to use for driving
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Gamepad driver, BunyipsSubsystem drive, Supplier<List<ContourData>> supplier, PIDF translationController, PIDF rotationController) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, supplier, translationController, rotationController);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param supplier              a supplier source that will provide contour data
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Measure<Time> timeout, BunyipsSubsystem drive, Supplier<List<ContourData>> supplier, PIDF translationController, PIDF rotationController) {
        super(timeout);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        onSubsystem(drive, false);
        this.drive = (RoadRunnerDrive) drive;
        contours = supplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.getPIDFController().updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.getPIDFController().updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
    }

    /**
     * TeleOp constructor.
     *
     * @param xSupplier             x (strafe) value
     * @param ySupplier             y (forward) value
     * @param rSupplier             r (rotate) value
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, BunyipsSubsystem drive, Processor<ContourData> processor, PIDF translationController, PIDF rotationController) {
        this(xSupplier, ySupplier, rSupplier, drive, processor::getData, translationController, rotationController);
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver                the gamepad to use for driving
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Gamepad driver, BunyipsSubsystem drive, Processor<ContourData> processor, PIDF translationController, PIDF rotationController) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, processor::getData, translationController, rotationController);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Measure<Time> timeout, BunyipsSubsystem drive, Processor<ContourData> processor, PIDF translationController, PIDF rotationController) {
        this(timeout, drive, processor::getData, translationController, rotationController);
    }

    /**
     * Set the pitch target of where the pixel should be on the camera. 0.0 is the middle.
     *
     * @param pitchTarget the target pitch to move to
     * @return the task
     */
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

        Pose2d pose = new Pose2d(0, 0, 0);
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<ContourData> data = contours.get();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -translationController.calculate(biggestContour.getPitch(), Range.clip(PITCH_TARGET, -1.0, 1.0)),
                            pose.getY(),
                            rotationController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
            hasCalculated = true;
        } else {
            drive.setWeightedDrivePower(pose);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return x == null && hasCalculated && translationController.getPIDFController().atSetPoint() && rotationController.getPIDFController().atSetPoint();
    }
}
