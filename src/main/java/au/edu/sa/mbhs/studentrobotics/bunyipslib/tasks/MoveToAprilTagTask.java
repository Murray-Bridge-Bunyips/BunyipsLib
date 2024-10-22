package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

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
    @NonNull
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
    private Supplier<PoseVelocity2d> passthrough;

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
    public MoveToAprilTagTask(@NonNull Measure<Time> timeout, @NonNull Moveable drive, @NonNull AprilTag aprilTag, @SuppressLint("LambdaLast") int targetTag) {
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
     * @param passthrough the pose velocity passthrough for the drivetrain
     * @param drive       the drivetrain to use, which may be a BunyipsSubsystem that will auto-attach
     * @param aprilTag    the AprilTag processor to use
     * @param targetTag   the tag to target. -1 for any tag
     */
    public MoveToAprilTagTask(@NonNull Supplier<PoseVelocity2d> passthrough, @NonNull Moveable drive, @NonNull AprilTag aprilTag, @SuppressLint("LambdaLast") int targetTag) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        this.aprilTag = aprilTag;
        this.passthrough = passthrough;
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
        DESIRED_DISTANCE = desiredDistance;
        return this;
    }

    /**
     * Set the forward speed gain for the distance error.
     *
     * @param speedGain the speed gain for the distance error
     * @return this
     */
    @NonNull
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
    @NonNull
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
    @NonNull
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
    @NonNull
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
    @NonNull
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
    @NonNull
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
        if (!target.isPresent() || !target.get().isInLibrary()) {
            drive.setPower(vel);
            return;
        }

        assert target.get().getFtcPose().isPresent() && target.get().getMetadata().isPresent();
        AprilTagPoseFtc camPose = target.get().getFtcPose().get();
        rangeError = (camPose.range - DESIRED_DISTANCE.in(Inches)) * SPEED_GAIN;
        yawError = -camPose.yaw * STRAFE_GAIN;
        headingError = camPose.bearing * TURN_GAIN;

        drive.setPower(Geometry.vel(
                Range.clip(rangeError, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                Range.clip(yawError, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                Range.clip(headingError, -MAX_AUTO_TURN, MAX_AUTO_TURN)
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
        return passthrough == null && Math.abs(rangeError) < AUTO_FINISH_ERROR_TOLERANCE && Math.abs(yawError) < AUTO_FINISH_ERROR_TOLERANCE && Math.abs(headingError) < AUTO_FINISH_ERROR_TOLERANCE;
    }
}
