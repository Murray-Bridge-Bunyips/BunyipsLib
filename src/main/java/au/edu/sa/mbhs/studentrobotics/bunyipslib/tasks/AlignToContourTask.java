package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.ContourData;

/**
 * Task to align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class AlignToContourTask extends Task {
    /**
     * The tolerance for the R error to be considered at the setpoint.
     */
    public static double R_TOLERANCE = 0.1;
    /**
     * Default controller to use for the rotation axis.
     */
    @NonNull
    public static PIDFController DEFAULT_CONTROLLER = new PDController(1, 0.0001);

    private final Moveable drive;
    private final Supplier<List<ContourData>> contours;
    private final Supplier<PoseVelocity2d> passthrough;
    private SystemController controller;
    private Double yaw;

    /**
     * TeleOp constructor.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier    a supplier source that will provide contour data
     */
    public AlignToContourTask(@Nullable Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        contours = supplier;
        this.passthrough = passthrough;
        controller = DEFAULT_CONTROLLER;
        withName("Align To Contour");
        Dashboard.enableConfig(getClass());
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver   the gamepad to use for driving
     * @param drive    the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier a supplier source that will provide contour data
     */
    public AlignToContourTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, supplier);
    }

    /**
     * Autonomous constructor.
     *
     * @param drive    the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param supplier a supplier source that will provide contour data
     */
    public AlignToContourTask(@NonNull Moveable drive, @NonNull Supplier<List<ContourData>> supplier) {
        this((Supplier<PoseVelocity2d>) null, drive, supplier);
    }

    /**
     * Set the controller for the rotation axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public AlignToContourTask withController(@NonNull SystemController r) {
        controller = r;
        return this;
    }

    @Override
    protected void init() {
        yaw = null;
    }

    @Override
    protected void periodic() {
        PoseVelocity2d vel = Geometry.zeroVel();
        if (passthrough != null)
            vel = passthrough.get();

        List<ContourData> data = contours.get();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            yaw = biggestContour.getYaw();
            drive.setPower(new PoseVelocity2d(
                    vel.linearVel,
                    controller.calculate(yaw, 0.0)
            ));
        } else {
            drive.setPower(vel);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        return passthrough == null && yaw != null && Math.abs(yaw) < R_TOLERANCE;
    }
}
