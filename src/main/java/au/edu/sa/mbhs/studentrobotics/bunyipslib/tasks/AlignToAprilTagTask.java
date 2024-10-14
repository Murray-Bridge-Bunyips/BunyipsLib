package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Drawing;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

/**
 * Task to align to an AprilTag.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
@Config
public class AlignToAprilTagTask extends Task {
    /**
     * PIDF coefficients for the alignment controller.
     */
    public static PIDFCoefficients coeffs = new PIDFCoefficients();
    /**
     * The target tag to align to. -1 for any tag.
     */
    public static int TARGET_TAG = -1;

    private final Moveable drive;
    private final AprilTag at;
    private final PIDF controller;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;
    private boolean hasCalculated;

    /**
     * Autonomous constructor.
     *
     * @param timeout    the timeout for the task
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at         the AprilTag processor to use
     * @param targetTag  the tag to align to, -1 for any tag
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(Measure<Time> timeout, Moveable drive, AprilTag at, int targetTag, PIDF controller) {
        super(timeout);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.at = at;
        TARGET_TAG = targetTag;
        this.controller = controller;
        controller.getPIDFController().setPIDF(coeffs);
        withName("Align To AprilTag");
    }

    /**
     * TeleOp constructor.
     *
     * @param xSupplier  x (strafe) value
     * @param ySupplier  y (forward) value
     * @param rSupplier  r (rotate) value
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at         the AprilTag processor to use
     * @param targetTag  the tag to align to, -1 for any tag
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, Moveable drive, AprilTag at, int targetTag, PIDF controller) {
        super(INFINITE_TIMEOUT);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.at = at;
        TARGET_TAG = targetTag;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.controller = controller;
        controller.getPIDFController().updatePIDF(coeffs);
        withName("Align To AprilTag");
    }

    /**
     * Constructor for AlignToAprilTagTask using a default Mecanum binding.
     *
     * @param driver     The gamepad to use for driving
     * @param drive      the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param at         The AprilTag processor to use
     * @param targetTag  The tag to align to, -1 for any tag
     * @param controller The PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(Gamepad driver, Moveable drive, AprilTag at, int targetTag, PIDF controller) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, at, targetTag, controller);
    }

    @Override
    protected void init() {
        hasCalculated = false;
        if (!at.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.getPIDFController().setPIDF(coeffs);

        Pose2d pose = Geometry.zeroPose();
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<AprilTagData> data = at.getData();

        Optional<AprilTagData> target = data.stream().filter(t -> TARGET_TAG == -1 || t.getId() == TARGET_TAG).findFirst();

        if (!target.isPresent() || !target.get().isInLibrary()) {
            drive.setPower(Geometry.poseToVel(pose));
            return;
        }

        assert target.get().getFtcPose().isPresent() && target.get().getMetadata().isPresent();
        drive.setPower(Geometry.poseToVel(
                new Pose2d(
                        pose.position.x,
                        pose.position.y,
                        -controller.calculate(target.get().getFtcPose().get().bearing, 0.0)
                )
        ));
        hasCalculated = true;

        Pose2d poseEstimate = drive.getPoseEstimate();
        if (poseEstimate == null)
            throw new IllegalStateException("A drive localizer must be present to use AlignToAprilTagTask!");
        VectorF point = target.get().getMetadata().get().fieldPosition;

        Drawing.useCanvas(canvas -> canvas.setStroke("#dd2c00")
                .strokeCircle(point.get(0), point.get(1), 2)
                .setStroke("#b89eff")
                .strokeLine(point.get(0), point.get(1), poseEstimate.position.x, poseEstimate.position.y)
                .setStroke("#ffce7a")
                .strokeLine(point.get(0), point.get(1), point.get(0), poseEstimate.position.y)
                .strokeLine(point.get(0), poseEstimate.position.y, poseEstimate.position.x, poseEstimate.position.y));
    }

    @Override
    protected boolean isTaskFinished() {
        return x == null && hasCalculated && controller.getPIDFController().atSetPoint();
    }
}
