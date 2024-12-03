package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

/**
 * Task to move to and align to an AprilTag.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class MoveToAprilTagTask extends Task {
    /**
     * The desired distance from the tag.
     */
    public static double DESIRED_DISTANCE_INCHES = Meters.of(1).in(Inches);
    /**
     * The maximum speed the robot can move at.
     */
    public static double MAX_X_SPEED = 0.5;
    /**
     * The maximum strafe the robot can move at.
     */
    public static double MAX_Y_SPEED = 0.5;
    /**
     * The maximum turn the robot can move at.
     */
    public static double MAX_R_SPEED = 0.3;
    /**
     * Tolerance of the X error for the robot to stop the task in an Autonomous context.
     */
    public static double X_TOLERANCE = 0.1;
    /**
     * Tolerance of the Y error for the robot to stop the task in an Autonomous context.
     */
    public static double Y_TOLERANCE = 0.1;
    /**
     * Tolerance of the R error for the robot to stop the task in an Autonomous context.
     */
    public static double R_TOLERANCE = 0.1;
    /**
     * The tag to target. -1 for any tag.
     */
    public static int TARGET_TAG = -1;
    /**
     * Default X controller
     */
    @NonNull
    public static PDController DEFAULT_X_CONTROLLER = new PDController(1, 0.0001);
    /**
     * Default Y controller
     */
    @NonNull
    public static PDController DEFAULT_Y_CONTROLLER = new PDController(1, 0.0001);
    /**
     * Default R controller
     */
    @NonNull
    public static PDController DEFAULT_R_CONTROLLER = new PDController(0.1, 0.0001);

    private final Moveable drive;
    private final AprilTag aprilTag;

    private Supplier<PoseVelocity2d> passthrough;
    private SystemController xController;
    private SystemController yController;
    private SystemController rController;

    private double rangeError;
    private double yawError;
    private double headingError;

    /**
     * Autonomous constructor.
     *
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag  the AprilTag processor to use
     * @param targetTag the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(@NonNull Moveable drive, @NonNull AprilTag aprilTag, @SuppressLint("LambdaLast") int targetTag) {
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.aprilTag = aprilTag;
        TARGET_TAG = targetTag;
        xController = DEFAULT_X_CONTROLLER;
        yController = DEFAULT_Y_CONTROLLER;
        rController = DEFAULT_R_CONTROLLER;
        named("Move to AprilTag");
        Dashboard.enableConfig(getClass());
    }

    /**
     * TeleOp constructor with default values.
     *
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag    the AprilTag processor to use
     * @param targetTag   the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull AprilTag aprilTag, @SuppressLint("LambdaLast") int targetTag) {
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.aprilTag = aprilTag;
        this.passthrough = passthrough;
        TARGET_TAG = targetTag;
        xController = DEFAULT_X_CONTROLLER;
        yController = DEFAULT_Y_CONTROLLER;
        rController = DEFAULT_R_CONTROLLER;
        named("Move to AprilTag");
        Dashboard.enableConfig(getClass());
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param gamepad   the gamepad to use for driving
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag  the AprilTag processor to use
     * @param targetTag the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(@NonNull Gamepad gamepad, @NonNull Moveable drive, @NonNull AprilTag aprilTag, @SuppressLint("LambdaLast") int targetTag) {
        this(() -> Controls.vel(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x), drive, aprilTag, targetTag);
    }

    /**
     * Set the desired distance from the tag.
     *
     * @param desiredDistance the desired distance from the tag
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withDesiredDistance(@NonNull Measure<Distance> desiredDistance) {
        DESIRED_DISTANCE_INCHES = desiredDistance.in(Inches);
        return this;
    }

    /**
     * Set the controller for the x (forward) axis.
     *
     * @param x the controller to use
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withXController(@NonNull SystemController x) {
        xController = x;
        return this;
    }

    /**
     * Set the controller for the y (strafe) axis.
     *
     * @param y the controller to use
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withYController(@NonNull SystemController y) {
        yController = y;
        return this;
    }

    /**
     * Set the controller for the r (rotation) axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withRController(@NonNull SystemController r) {
        rController = r;
        return this;
    }

    /**
     * Set the maximum speed the robot can move at.
     *
     * @param maxAutoSpeed the maximum speed the robot can move at
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withMaxXSpeed(double maxAutoSpeed) {
        MAX_X_SPEED = maxAutoSpeed;
        return this;
    }

    /**
     * Set the maximum strafe the robot can move at.
     *
     * @param maxAutoStrafe the maximum strafe the robot can move at
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withMaxYSpeed(double maxAutoStrafe) {
        MAX_Y_SPEED = maxAutoStrafe;
        return this;
    }

    /**
     * Set the maximum turn the robot can move at.
     *
     * @param maxAutoTurn the maximum turn the robot can move at
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withMaxRSpeed(double maxAutoTurn) {
        MAX_R_SPEED = maxAutoTurn;
        return this;
    }

    /**
     * Set the tag ID to target. -1 for any tag.
     *
     * @param targetTag the tag ID to target
     * @return this
     */
    @NonNull
    public MoveToAprilTagTask withTargetTag(int targetTag) {
        TARGET_TAG = targetTag;
        return this;
    }

    @Override
    protected void init() {
        if (!aprilTag.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        PoseVelocity2d vel = passthrough.get();

        List<AprilTagData> data = aprilTag.getData();

        Optional<AprilTagData> target = data.stream().filter(t -> TARGET_TAG == -1 || t.getId() == TARGET_TAG).findFirst();
        if (target.isEmpty() || !target.get().isInLibrary()) {
            drive.setPower(vel);
            return;
        }

        assert target.get().getFtcPose().isPresent() && target.get().getMetadata().isPresent();
        AprilTagPoseFtc camPose = target.get().getFtcPose().get();
        rangeError = camPose.range - DESIRED_DISTANCE_INCHES;
        yawError = -camPose.yaw;
        headingError = camPose.bearing;

        drive.setPower(Geometry.vel(
                Range.clip(xController.calculate(rangeError, 0), -MAX_X_SPEED, MAX_X_SPEED),
                Range.clip(yController.calculate(yawError, 0), -MAX_Y_SPEED, MAX_Y_SPEED),
                Range.clip(-rController.calculate(headingError, 0), -MAX_R_SPEED, MAX_R_SPEED)
        ));

        Pose2d poseEstimate = drive.getPose();
        if (poseEstimate == null)
            throw new IllegalStateException("A drive localizer must be present to use MoveToAprilTagTask!");
        VectorF point = target.get().getMetadata().get().fieldPosition;

        fieldOverlay.setStroke("#dd2c00")
                .strokeCircle(point.get(0), point.get(1), 2)
                .setStroke("#b89eff")
                .strokeLine(point.get(0), point.get(1), poseEstimate.position.x, poseEstimate.position.y)
                .setStroke("#ffce7a")
                .strokeLine(point.get(0), point.get(1), point.get(0), poseEstimate.position.y)
                .strokeLine(point.get(0), poseEstimate.position.y, poseEstimate.position.x, poseEstimate.position.y);
    }

    @Override
    protected boolean isTaskFinished() {
        return passthrough == null && Math.abs(rangeError) < X_TOLERANCE && Math.abs(yawError) < Y_TOLERANCE && Math.abs(headingError) < R_TOLERANCE;
    }
}
