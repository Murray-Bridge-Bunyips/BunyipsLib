package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * Task to move to and align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class MoveToContourTask extends Task {
    /**
     * The target position of the pixel on the camera pitch axis.
     */
    public static double PITCH_TARGET = 0.0;
    /**
     * The tolerance of the pitch target to the real pitch.
     */
    public static double X_TOLERANCE = 0.01;
    /**
     * The tolerance of the yaw target to the real yaw.
     */
    public static double R_TOLERANCE = 0.01;
    /**
     * Default controller to use for the x (forward) axis.
     */
    @NonNull
    public static SystemController DEFAULT_X_CONTROLLER = new PDController(1, 0.0001);
    /**
     * Default controller to use for the r (rotation) axis.
     */
    @NonNull
    public static SystemController DEFAULT_R_CONTROLLER = new PDController(1, 0.0001);

    private final Moveable drive;
    private final Supplier<List<ContourData>> contours;
    private final Supplier<PoseVelocity2d> passthrough;

    private SystemController xController;
    private SystemController rController;
    private ContourData biggestContour;

    /**
     * TeleOp constructor.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier    a supplier source that will provide contour data
     */
    public MoveToContourTask(@Nullable Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.passthrough = passthrough;
        xController = DEFAULT_X_CONTROLLER;
        rController = DEFAULT_R_CONTROLLER;
        withName("Move to Contour");
        Dashboard.enableConfig(getClass());
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver   the gamepad to use for driving
     * @param drive    the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier a supplier source that will provide contour data
     */
    public MoveToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, supplier);
    }

    /**
     * Autonomous constructor.
     *
     * @param drive    the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier a supplier source that will provide contour data
     */
    public MoveToContourTask(@NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        this((Supplier<PoseVelocity2d>) null, drive, supplier);
    }

    /**
     * Set the controller for the x (forward) axis.
     *
     * @param x the controller to use
     * @return this
     */
    @NonNull
    public MoveToContourTask withXController(@NonNull SystemController x) {
        xController = x;
        return this;
    }

    /**
     * Set the controller for the r (rotation) axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public MoveToContourTask withRController(@NonNull SystemController r) {
        rController = r;
        return this;
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
        biggestContour = null;
    }

    @Override
    protected void periodic() {
        PoseVelocity2d vel = Geometry.zeroVel();
        if (passthrough != null)
            vel = passthrough.get();

        List<ContourData> data = contours.get();
        biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setPower(
                    Geometry.vel(
                            -xController.calculate(biggestContour.getPitch(), Mathf.clamp(PITCH_TARGET, -1.0, 1.0)),
                            vel.linearVel.y,
                            rController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
        } else {
            drive.setPower(vel);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return passthrough == null && biggestContour != null && Mathf.isNear(biggestContour.getPitch(), PITCH_TARGET, X_TOLERANCE) && Mathf.isNear(biggestContour.getYaw(), 0.0, R_TOLERANCE);
    }
}
