package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
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
     * The tolerance of the X controller for Auto.
     */
    public static double X_TOLERANCE = 0.05;
    /**
     * The tolerance of the yaw target controller to the real yaw for Auto.
     */
    public static double R_TOLERANCE = 0.1;
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

    private Function<ContourData, Double> errorSupplier = (c) -> 0.5 - c.getPitch();
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
        withXController(DEFAULT_X_CONTROLLER);
        withRController(DEFAULT_R_CONTROLLER);
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
        xController.pidf().ifPresent(pid -> pid.setTolerance(X_TOLERANCE));
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
        rController.pidf().ifPresent(pid -> pid.setTolerance(R_TOLERANCE));
        return this;
    }

    /**
     * Sets a custom error supplier for the forward movement of the robot.
     *
     * @param forwardErrorSupplier the error that dictates the forward movement of the robot (whether by pitch or area, etc),
     *                             by default this is a pitch target of 0.5 (middle of frame)
     * @return the task
     */
    @NonNull
    public MoveToContourTask withForwardErrorSupplier(Function<ContourData, Double> forwardErrorSupplier) {
        errorSupplier = forwardErrorSupplier;
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
                            -xController.calculate(errorSupplier.apply(biggestContour), 0.0),
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
        boolean xSetpoint = true, rSetpoint = true;
        if (xController.pidf().isPresent()) {
            xSetpoint = xController.pidf().get().atSetPoint();
        }
        if (rController.pidf().isPresent()) {
            rSetpoint = rController.pidf().get().atSetPoint();
        }
        return passthrough == null && biggestContour != null && xSetpoint && rSetpoint;
    }
}
