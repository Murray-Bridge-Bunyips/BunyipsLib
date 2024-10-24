package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Centimeters;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;

import java.util.Objects;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
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
    private final Moveable drive;
    private final Pose2d targetPose;
    private final SystemController forwardController;
    private final SystemController strafeController;
    private final SystemController headingController;
    private final Supplier<Pose2d> localizer;

    private double MAX_FORWARD_SPEED = 1.0;
    private double MAX_STRAFE_SPEED = 1.0;
    private double MAX_ROTATION_SPEED = 1.0;

    private Measure<Angle> headingTolerance = Degrees.of(2);
    private Measure<Distance> vectorTolerance = Centimeters.of(5);

    /**
     * Run the Drive To Pose Task on a drive instance.
     *
     * @param timeout           The maximum time the task can run for.
     * @param driveInstance     The drive instance to run this task with. If this instance is also a BunyipsSubsystem, this task will be auto-attached.
     * @param targetPose        The target pose to drive to.
     * @param forwardController The system/PID controller for x.
     * @param strafeController  The system/PID controller for y.
     * @param headingController The system/PID controller for heading.
     */
    public DriveToPoseTask(@NonNull Measure<Time> timeout, @NonNull Moveable driveInstance,
                           @NonNull Pose2d targetPose, @NonNull SystemController forwardController, @NonNull SystemController strafeController, @SuppressLint("LambdaLast") @NonNull SystemController headingController) {
        super(timeout);
        if (driveInstance instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) driveInstance, true);
        drive = driveInstance;
        localizer = () -> Objects.requireNonNull(drive.getPose(), "A localizer must be attached to the drive instance for P2P to work!");
        this.targetPose = targetPose;
        this.forwardController = forwardController;
        this.strafeController = strafeController;
        this.headingController = headingController;
        withName("Drive To Pose: " + targetPose);
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
        return Mathf.isNear(0, Geometry.distBetween(localizer.get().position, targetPose.position), vectorTolerance.in(Inches));
    }

    public boolean isHeadingNear() {
        return Mathf.isNear(targetPose.heading.toDouble(), localizer.get().heading.toDouble(), headingTolerance.in(Radians));
    }

    @Override
    protected void periodic() {
        Pose2d estimatedPose = localizer.get();
        Twist2d error = targetPose.minus(estimatedPose);

        // Twist the error vector to be relative to the robot's heading, as rotations of the robot are not
        // accounted for in the RoadRunner pose estimate
        double cos = Math.cos(estimatedPose.heading.toDouble());
        double sin = Math.sin(estimatedPose.heading.toDouble());

        // Transform error vector to robot's coordinate frame
        double twistedXError = error.line.x * cos + error.line.y * sin;
        double twistedYError = -error.line.x * sin + error.line.y * cos;

        // Wrap target angle between -pi and pi for optimal turns
        double angleError = Mathf.wrap(error.angle, -Math.PI, Math.PI);
        // When the angle is near the modulus boundary, lock towards a definitive full rotation to avoid oscillations
        if (Mathf.isNear(Math.abs(angleError), Math.PI, 0.1))
            angleError = -Math.PI * Math.signum(error.angle);

        // Apply PID and twist
        double forwardPower = -forwardController.calculate(twistedXError, 0);
        double strafePower = -strafeController.calculate(twistedYError, 0);
        double headingPower = -headingController.calculate(angleError, 0);

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
