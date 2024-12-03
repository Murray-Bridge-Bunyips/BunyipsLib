package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Objects;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Drive to a pose using a localizer and PID-To-Point.
 * <p>
 * This is useful for pose alignment based on error, rather than using trajectories for planned motion.
 * Designed and tested for Mecanum drivebases as RoadRunner was designed around them.
 *
 * @author Lucas Bubner, 2024
 * @since 3.3.0
 */
public class DriveToPoseTask extends Task {
    /**
     * Default controller to use for the x (forward) axis.
     */
    @NonNull
    public static SystemController DEFAULT_X_CONTROLLER = new PDController(1, 0.0001);
    /**
     * Default controller to use for the y (strafe) axis.
     */
    @NonNull
    public static SystemController DEFAULT_Y_CONTROLLER = new PDController(1, 0.0001);
    /**
     * Default controller to use for the r (rotation) axis.
     */
    @NonNull
    public static SystemController DEFAULT_R_CONTROLLER = new PDController(3, 0.0001);

    private final Moveable drive;
    private final Pose2d targetPose;
    private final Supplier<Pose2d> accumulator;

    private double MAX_FORWARD_SPEED = 1.0;
    private double MAX_STRAFE_SPEED = 1.0;
    private double MAX_ROTATION_SPEED = 1.0;

    private SystemController xController;
    private SystemController yController;
    private SystemController rController;

    private Measure<Angle> headingTolerance = Degrees.one();
    private Measure<Distance> vectorTolerance = Inches.one();

    /**
     * Run the Drive To Pose Task on a drive instance.
     *
     * @param targetPose    The target pose to drive to.
     * @param driveInstance The drive instance to run this task with. If this instance is also a BunyipsSubsystem, this task will be auto-attached.
     */
    public DriveToPoseTask(@NonNull Pose2d targetPose, @NonNull Moveable driveInstance) {
        if (driveInstance instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) driveInstance, true);
        drive = driveInstance;
        accumulator = () -> Objects.requireNonNull(drive.getPose(), "A localizer must be attached to the drive instance for P2P to work!");
        this.targetPose = targetPose;
        xController = DEFAULT_X_CONTROLLER;
        yController = DEFAULT_Y_CONTROLLER;
        rController = DEFAULT_R_CONTROLLER;
        named("Drive To Pose: " + Geometry.toUserString(targetPose));
    }

    /**
     * Sets the controller for the x (forward) axis.
     *
     * @param x the controller to use
     * @return this
     */
    @NonNull
    public DriveToPoseTask withXController(@NonNull SystemController x) {
        xController = x;
        return this;
    }

    /**
     * Sets the controller for the y (strafe) axis.
     *
     * @param y the controller to use
     * @return this
     */
    @NonNull
    public DriveToPoseTask withYController(@NonNull SystemController y) {
        yController = y;
        return this;
    }

    /**
     * Sets the controller for the r (rotation) axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public DriveToPoseTask withRController(@NonNull SystemController r) {
        rController = r;
        return this;
    }

    /**
     * Set the tolerances for the task.
     *
     * @param heading     The tolerance for heading.
     * @param translation The tolerance for the translation vector.
     * @return this
     */
    @NonNull
    public DriveToPoseTask withTolerances(@NonNull Measure<Angle> heading, @NonNull Measure<Distance> translation) {
        headingTolerance = heading;
        vectorTolerance = translation;
        return this;
    }

    /**
     * Set the maximum forward (x) speed (motor power) that the robot can move at.
     *
     * @param speed The maximum forward speed magnitude.
     * @return this
     */
    @NonNull
    public DriveToPoseTask withMaxForwardSpeed(double speed) {
        MAX_FORWARD_SPEED = Math.abs(speed);
        return this;
    }

    /**
     * Set the maximum strafe (y) speed (motor power) that the robot can move at.
     *
     * @param speed The maximum strafe speed magnitude.
     * @return this
     */
    @NonNull
    public DriveToPoseTask withMaxStrafeSpeed(double speed) {
        MAX_STRAFE_SPEED = Math.abs(speed);
        return this;
    }

    /**
     * Set the maximum rotation speed (motor power) that the robot can move at.
     *
     * @param speed The maximum rotation speed magnitude.
     * @return this
     */
    @NonNull
    public DriveToPoseTask withMaxRotationSpeed(double speed) {
        MAX_ROTATION_SPEED = Math.abs(speed);
        return this;
    }

    @Override
    protected void init() {
        drive.setPower(Geometry.zeroVel());
    }

    public boolean isVectorNear() {
        return Mathf.isNear(0, Geometry.distTo(accumulator.get().position, targetPose.position), vectorTolerance.in(Inches));
    }

    public boolean isHeadingNear() {
        return Mathf.isNear(0, targetPose.heading.minus(accumulator.get().heading), headingTolerance.in(Radians));
    }

    @Override
    protected void periodic() {
        Pose2d estimatedPose = accumulator.get();
        Pose2d error = targetPose.minusExp(estimatedPose);

        // Apply PID and twist
        double forwardPower, strafePower;
        if (!isVectorNear()) {
            // Normalise vector to maintain the heading interpolation ratio
            double mag = Math.hypot(error.position.x, error.position.y);
            forwardPower = -xController.calculate(error.position.x / mag, 0);
            strafePower = -yController.calculate(error.position.y / mag, 0);
        } else {
            forwardPower = 0;
            strafePower = 0;
        }

        double headingPower;
        if (!isHeadingNear()) {
            headingPower = -rController.calculate(error.heading.toDouble(), 0);
        } else {
            headingPower = 0;
        }

        drive.setPower(Geometry.vel(
                Mathf.clamp(forwardPower, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED),
                Mathf.clamp(strafePower, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED),
                Mathf.clamp(headingPower, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED)
        ));

        fieldOverlay.setStroke("#c91c00")
                .strokeLine(estimatedPose.position.x, estimatedPose.position.y, targetPose.position.x, targetPose.position.y);
        fieldOverlay.setStroke("#4CAF50");
        Dashboard.drawRobot(fieldOverlay, targetPose);
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }

    @Override
    protected boolean isTaskFinished() {
        return isVectorNear() && isHeadingNear();
    }
}
