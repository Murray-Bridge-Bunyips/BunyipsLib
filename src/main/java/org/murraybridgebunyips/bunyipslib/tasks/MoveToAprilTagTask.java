package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Meters;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.drive.Moveable;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.DashboardUtil;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Task to move to and align to an AprilTag.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
@Config
public class MoveToAprilTagTask extends Task {
    /**
     * The desired distance from the tag.
     */
    public static Measure<Distance> DESIRED_DISTANCE = Meters.of(1);
    /**
     * The speed gain for the distance error.
     */
    public static double SPEED_GAIN = 0.03;
    /**
     * The strafe gain for the yaw error.
     */
    public static double STRAFE_GAIN = 0.015;
    /**
     * The turn gain for the heading error.
     */
    public static double TURN_GAIN = 0.01;
    /**
     * The maximum speed the robot can move at.
     */
    public static double MAX_AUTO_SPEED = 0.5;
    /**
     * The maximum strafe the robot can move at.
     */
    public static double MAX_AUTO_STRAFE = 0.5;
    /**
     * The maximum turn the robot can move at.
     */
    public static double MAX_AUTO_TURN = 0.3;
    /**
     * Tolerance of the error for the robot to stop the task in an Autonomous context.
     */
    public static double AUTO_FINISH_ERROR_TOLERANCE = 0.1;
    /**
     * The tag to target. -1 for any tag.
     */
    public static int TARGET_TAG = -1;

    private final Moveable drive;
    private final AprilTag aprilTag;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    private double rangeError;
    private double yawError;
    private double headingError;

    /**
     * Autonomous constructor.
     *
     * @param timeout   the timeout for the task
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag  the AprilTag processor to use
     * @param targetTag the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(Measure<Time> timeout, Moveable drive, AprilTag aprilTag, int targetTag) {
        super(timeout);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.aprilTag = aprilTag;
        TARGET_TAG = targetTag;
        withName("Move to AprilTag");
    }

    /**
     * TeleOp constructor with default values.
     *
     * @param xSupplier x (strafe) value
     * @param ySupplier y (forward) value
     * @param rSupplier r (rotate) value
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag  the AprilTag processor to use
     * @param targetTag the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, Moveable drive, AprilTag aprilTag, int targetTag) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.aprilTag = aprilTag;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        TARGET_TAG = targetTag;
        withName("Move to AprilTag");
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param gamepad   the gamepad to use for driving
     * @param drive     the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag  the AprilTag processor to use
     * @param targetTag the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(Gamepad gamepad, Moveable drive, AprilTag aprilTag, int targetTag) {
        this(() -> gamepad.left_stick_x, () -> gamepad.left_stick_y, () -> gamepad.right_stick_x, drive, aprilTag, targetTag);
    }

    /**
     * Set the desired distance from the tag.
     *
     * @param desiredDistance the desired distance from the tag
     * @return this
     */
    public MoveToAprilTagTask withDesiredDistance(Measure<Distance> desiredDistance) {
        DESIRED_DISTANCE = desiredDistance;
        return this;
    }

    /**
     * Set the forward speed gain for the distance error.
     *
     * @param speedGain the speed gain for the distance error
     * @return this
     */
    public MoveToAprilTagTask withSpeedGain(double speedGain) {
        SPEED_GAIN = speedGain;
        return this;
    }

    /**
     * Set the strafe gain for the yaw error.
     *
     * @param strafeGain the strafe gain for the yaw error
     * @return this
     */
    public MoveToAprilTagTask withStrafeGain(double strafeGain) {
        STRAFE_GAIN = strafeGain;
        return this;
    }

    /**
     * Set the turn gain for the heading error.
     *
     * @param turnGain the turn gain for the heading error
     * @return this
     */
    public MoveToAprilTagTask withTurnGain(double turnGain) {
        TURN_GAIN = turnGain;
        return this;
    }

    /**
     * Set the maximum speed the robot can move at.
     *
     * @param maxAutoSpeed the maximum speed the robot can move at
     * @return this
     */
    public MoveToAprilTagTask withMaxAutoSpeed(double maxAutoSpeed) {
        MAX_AUTO_SPEED = maxAutoSpeed;
        return this;
    }

    /**
     * Set the maximum strafe the robot can move at.
     *
     * @param maxAutoStrafe the maximum strafe the robot can move at
     * @return this
     */
    public MoveToAprilTagTask withMaxAutoStrafe(double maxAutoStrafe) {
        MAX_AUTO_STRAFE = maxAutoStrafe;
        return this;
    }

    /**
     * Set the maximum turn the robot can move at.
     *
     * @param maxAutoTurn the maximum turn the robot can move at
     * @return this
     */
    public MoveToAprilTagTask withMaxAutoTurn(double maxAutoTurn) {
        MAX_AUTO_TURN = maxAutoTurn;
        return this;
    }

    /**
     * Set the tag ID to target. -1 for any tag.
     *
     * @param targetTag the tag ID to target
     * @return this
     */
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
        Pose2d pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<AprilTagData> data = aprilTag.getData();

        Optional<AprilTagData> target = data.stream().filter(p -> TARGET_TAG == -1 || p.getId() == TARGET_TAG).findFirst();
        if (!target.isPresent() || !target.get().isInLibrary()) {
            drive.setPower(pose);
            return;
        }

        assert target.get().getFtcPose().isPresent() && target.get().getMetadata().isPresent();
        AprilTagPoseFtc camPose = target.get().getFtcPose().get();
        rangeError = (camPose.range - DESIRED_DISTANCE.in(Inches)) * SPEED_GAIN;
        yawError = -camPose.yaw * STRAFE_GAIN;
        headingError = camPose.bearing * TURN_GAIN;

        drive.setPower(
                new Pose2d(
                        Range.clip(rangeError, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                        Range.clip(yawError, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                        Range.clip(headingError, -MAX_AUTO_TURN, MAX_AUTO_TURN)
                )
        );

        if (drive.getLocalizer() == null)
            throw new IllegalStateException("A drive localizer must be present to use MoveToAprilTagTask!");
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
        VectorF point = target.get().getMetadata().get().fieldPosition;

        DashboardUtil.useCanvas(canvas -> canvas.setStroke("#dd2c00")
                .strokeCircle(point.get(0), point.get(1), 2)
                .setStroke("#b89eff")
                .strokeLine(point.get(0), point.get(1), poseEstimate.getX(), poseEstimate.getY())
                .setStroke("#ffce7a")
                .strokeLine(point.get(0), point.get(1), point.get(0), poseEstimate.getY())
                .strokeLine(point.get(0), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY()));
    }

    @Override
    protected boolean isTaskFinished() {
        return x == null && Math.abs(rangeError) < AUTO_FINISH_ERROR_TOLERANCE && Math.abs(yawError) < AUTO_FINISH_ERROR_TOLERANCE && Math.abs(headingError) < AUTO_FINISH_ERROR_TOLERANCE;
    }
}
