package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

/**
 * Task to align to an AprilTag.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class AlignToAprilTagTask extends FieldOrientableDriveTask {
    /**
     * Tolerance for the R error to be considered at the setpoint.
     */
    public static double R_TOLERANCE = 0.1;
    /**
     * The target tag to align to. -1 for any tag.
     */
    public static int TARGET_TAG = -1;
    /**
     * Default controller to use for the rotation axis.
     */
    @NonNull
    public static PIDFController DEFAULT_CONTROLLER = new PDController(0.1, 0.0001);

    private final AprilTag at;
    private final Supplier<PoseVelocity2d> vel;

    private SystemController controller;
    private Double bearing;

    /**
     * Autonomous constructor.
     *
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at        the AprilTag processor to use
     * @param targetTag the tag to align to, -1 for any tag
     */
    public AlignToAprilTagTask(@NonNull Moveable drive, @NonNull AprilTag at, int targetTag) {
        this((Supplier<PoseVelocity2d>) null, drive, at, targetTag);
    }

    /**
     * TeleOp constructor.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at          the AprilTag processor to use
     * @param targetTag   the tag to align to, -1 for any tag
     */
    public AlignToAprilTagTask(@Nullable Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull AprilTag at, int targetTag) {
        super.drive = drive;
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        this.at = at;
        TARGET_TAG = targetTag;
        vel = passthrough;
        controller = DEFAULT_CONTROLLER;
        named("Align To AprilTag");
        Dashboard.enableConfig(getClass());
    }

    /**
     * Constructor for AlignToAprilTagTask using a default Mecanum binding.
     *
     * @param driver    The gamepad to use for driving
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at        The AprilTag processor to use
     * @param targetTag The tag to align to, -1 for any tag
     */
    public AlignToAprilTagTask(@NonNull Gamepad driver, @NonNull Moveable drive, @NonNull AprilTag at, int targetTag) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive, at, targetTag);
    }

    /**
     * Set the controller for the rotation axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public AlignToAprilTagTask withRController(@NonNull SystemController r) {
        controller = r;
        return this;
    }

    @Override
    protected void init() {
        bearing = null;
        if (!at.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        PoseVelocity2d vel = Geometry.zeroVel();
        if (this.vel != null)
            vel = applyOrientation(this.vel.get());

        List<AprilTagData> data = at.getData();

        Optional<AprilTagData> target = data.stream().filter(t -> TARGET_TAG == -1 || t.getId() == TARGET_TAG).findFirst();

        if (target.isEmpty() || !target.get().isInLibrary()) {
            drive.setPower(vel);
            return;
        }

        assert target.get().getFtcPose().isPresent() && target.get().getMetadata().isPresent();
        bearing = target.get().getFtcPose().get().bearing;
        drive.setPower(new PoseVelocity2d(
                vel.linearVel,
                -controller.calculate(bearing, 0.0)
        ));

        Pose2d poseEstimate = drive.getPose();
        if (poseEstimate == null)
            throw new IllegalStateException("A drive localizer must be present to use AlignToAprilTagTask!");
        VectorF point = target.get().getMetadata().get().fieldPosition;

        dashboard.fieldOverlay().setStroke("#dd2c00")
                .strokeCircle(point.get(0), point.get(1), 2)
                .setStroke("#b89eff")
                .strokeLine(point.get(0), point.get(1), poseEstimate.position.x, poseEstimate.position.y)
                .setStroke("#ffce7a")
                .strokeLine(point.get(0), point.get(1), point.get(0), poseEstimate.position.y)
                .strokeLine(point.get(0), poseEstimate.position.y, poseEstimate.position.x, poseEstimate.position.y);
    }

    @Override
    protected boolean isTaskFinished() {
        return vel == null && bearing != null && Mathf.isNear(bearing, 0, R_TOLERANCE);
    }
}
