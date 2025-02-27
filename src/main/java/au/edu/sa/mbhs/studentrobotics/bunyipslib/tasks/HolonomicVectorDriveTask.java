package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Gamepad drive for all holonomic drivetrains which will use a vector lock based approach to drive.
 * This task is designed to be used as a default/standard-priority task, other tasks will override it.
 * <p>
 * Compared to {@link HolonomicDriveTask}, this task will constantly track the (x,y,r) pose of the robot, and will
 * set the drive powers directly from the gamepad inputs. When the translational/rotational components for these vectors
 * are zero, this task will take a snapshot of the values they were at, using PID controllers to attempt to stay in place.
 * This allows for more predictable and consistent driving, as the robot will only accept movement when told to do so,
 * ensuring that all movements of the robot can only be achieved via the controller.
 * <p>
 * This system can be comparable to one of a drone, where releasing the sticks and allowing it to hover will hold position
 * and resist external forces. This locking nature has been implemented on the vector (translation) and heading components.
 * Note that user input overrides all, where the x or y inputs will both unlock both translational axes to avoid corrections that could be dangerous.
 * This caveat means that this task will make no effort to try and correct translational deviation when the robot is being commanded.
 * <p>
 * A localizer-attached holonomic drive is required for this class, as it will require the use of the pose estimate system.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class HolonomicVectorDriveTask extends FieldOrientableDriveTask {
    /**
     * Default controller to use for the x (forward) axis.
     */
    @NonNull
    public static SystemController DEFAULT_X_CONTROLLER = new PController(0.1);
    /**
     * Default controller to use for the y (strafe) axis.
     */
    @NonNull
    public static SystemController DEFAULT_Y_CONTROLLER = new PController(0.1);
    /**
     * Default controller to use for the r (rotation) axis.
     */
    @NonNull
    public static SystemController DEFAULT_R_CONTROLLER = new PDController(1, 0.0001);

    private final Supplier<PoseVelocity2d> vel;
    private final ElapsedTime vectorLocker = new ElapsedTime();
    private final ElapsedTime headingLocker = new ElapsedTime();
    private SystemController xController;
    private SystemController yController;
    private SystemController rController;
    private Vector2d vectorLock = null;
    private Rotation2d headingLock = null;

    // Default admissible error of 1 inch and 1 degree, waiting 300ms for the pose to stabilise
    private Pose2d toleranceInchRad = new Pose2d(1, 1, Math.toRadians(1));
    private Measure<Time> lockingTimeout = Milliseconds.of(300);

    /**
     * Constructor for HolonomicTrackingDriveTask.
     *
     * @param targetVecNormalised The supplier for the current pose velocity of the robot, magnitude [-1, 1] of max robot velocity
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible. A localizer attached is required.
     */
    public HolonomicVectorDriveTask(@NonNull Supplier<PoseVelocity2d> targetVecNormalised, @NonNull Moveable drive) {
        super.drive = drive;
        if (drive instanceof BunyipsSubsystem)
            on((BunyipsSubsystem) drive, false);
        this.vel = targetVecNormalised;

        xController = DEFAULT_X_CONTROLLER;
        yController = DEFAULT_Y_CONTROLLER;
        rController = DEFAULT_R_CONTROLLER;

        named("Holonomic Vector Control");
    }

    /**
     * Constructor for HolonomicTrackingDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver The gamepad to use for driving
     * @param drive  The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *               called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *               if possible. A localizer attached is required.
     */
    public HolonomicVectorDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive);
    }

    /**
     * Sets the controller for the x (forward) axis.
     *
     * @param x the controller to use
     * @return this
     */
    @NonNull
    public HolonomicVectorDriveTask withXController(@NonNull SystemController x) {
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
    public HolonomicVectorDriveTask withYController(@NonNull SystemController y) {
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
    public HolonomicVectorDriveTask withRController(@NonNull SystemController r) {
        rController = r;
        return this;
    }

    /**
     * Set the minimum pose error when in self-holding mode to activate correction for.
     *
     * @param inchRad a pose in inches and radians that represents the admissible error for robot autocorrection
     * @return this
     */
    @NonNull
    public HolonomicVectorDriveTask withTolerance(@NonNull Pose2d inchRad) {
        toleranceInchRad = inchRad;
        return this;
    }

    /**
     * Set the pose stabilisation timeout before locking the pose vectors for correction.
     * This timeout applies for vector locking and heading locking individually.
     *
     * @param lockTimeout the time to wait when an axis magnitude is zero before locking, higher
     *                    values will yield more stable poses, but slower time to lock
     * @return this
     */
    @NonNull
    public HolonomicVectorDriveTask withStabilisationTimeout(@NonNull Measure<Time> lockTimeout) {
        lockingTimeout = lockTimeout;
        return this;
    }

    /**
     * Set the minimum pose error when in self-holding mode to activate correction for.
     *
     * @param poseX x (forward) admissible error
     * @param poseY y (strafe) admissible error
     * @param poseR r (heading) admissible error
     * @return this
     */
    @NonNull
    public HolonomicVectorDriveTask withTolerance(@NonNull Measure<Distance> poseX, @NonNull Measure<Distance> poseY, @NonNull Measure<Angle> poseR) {
        return withTolerance(new Pose2d(poseX.in(Inches), poseY.in(Inches), poseR.in(Radians)));
    }

    /**
     * Set a manual heading that this task should lock to, much as if the robot were rotated to this position
     * and is to try and hold rotation. Note that user input will override this lock, this
     * simply tells the task to respect this value as the locking value.
     *
     * @param heading the angle to rotate to, will be wrapped from [0, 2pi] radians
     */
    public void setHeadingTarget(@NonNull Measure<Angle> heading) {
        headingLock = Rotation2d.exp(Mathf.wrap(heading).in(Radians));
    }

    /**
     * Set a manual target that this task should lock to, much as if the robot was moved here
     * and is to try and hold position here. Note that user input will override this lock, this
     * simply tells the task to respect this value as the locking value.
     *
     * @param vectorInches the target position to move to, in inches
     */
    public void setVectorTarget(@NonNull Vector2d vectorInches) {
        vectorLock = vectorInches;
    }

    @Override
    protected void init() {
        vectorLocker.reset();
        headingLocker.reset();
        vectorLock = null;
        headingLock = null;
    }

    @Override
    protected void periodic() {
        Pose2d current = Objects.requireNonNull(drive.getPose(), "A localizer must be attached to the drive instance in order to use the HolonomicTrackingDriveTask!");

        PoseVelocity2d v = applyOrientation(vel.get());

        // Rising edge detections for pose locking
        if (v.linearVel.x == 0 && v.linearVel.y == 0 && vectorLock == null && vectorLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            vectorLock = current.position;
            // We also reset the controllers as the integral term may be incorrect due to a new target
            xController.reset();
            yController.reset();
        } else if (v.linearVel.x != 0 || v.linearVel.y != 0) {
            vectorLock = null;
            vectorLocker.reset();
        }
        if (v.angVel == 0 && headingLock == null && headingLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            headingLock = current.heading;
            rController.reset();
        } else if (v.angVel != 0) {
            headingLock = null;
            headingLocker.reset();
        }

        // Calculate error from current pose to target pose.
        // If we are not locked, the error will be 0, and our error should clamp to zero if it's under the threshold
        double xLockedError = vectorLock == null || Mathf.isNear(vectorLock.x, current.position.x, toleranceInchRad.position.x) ? 0 : vectorLock.x - current.position.x;
        double yLockedError = vectorLock == null || Mathf.isNear(vectorLock.y, current.position.y, toleranceInchRad.position.y) ? 0 : vectorLock.y - current.position.y;
        double rLockedError = headingLock == null || Mathf.isNear(headingLock.minus(current.heading), 0, toleranceInchRad.heading.toDouble()) ? 0 : headingLock.minus(current.heading);
        Vector2d rotError = current.heading.inverse().times(new Vector2d(xLockedError, yLockedError));
        drive.setPower(Geometry.vel(
                vectorLock != null ? -xController.calculate(rotError.x, 0) : v.linearVel.x,
                vectorLock != null ? -yController.calculate(rotError.y, 0) : v.linearVel.y,
                headingLock != null ? -rController.calculate(rLockedError, 0) : v.angVel
        ));

        dashboard.fieldOverlay().setStroke("#4CAF50BB");
        if (vectorLock != null)
            dashboard.fieldOverlay().strokeLine(current.position.x, current.position.y, vectorLock.x, vectorLock.y);
        dashboard.fieldOverlay().setStroke("#4CAF50");
        if (headingLock != null)
            Dashboard.drawRobot(dashboard.fieldOverlay(), new Pose2d(vectorLock != null ? vectorLock : current.position, headingLock));
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}

