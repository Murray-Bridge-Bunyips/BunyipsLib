package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ForeverTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * Gamepad drive for all holonomic drivetrains which will use a vector-based approach to drive.
 * This task is designed to be used as a default/standard-priority task, other tasks will override it.
 * <p>
 * Compared to {@link HolonomicDriveTask}, this task will constantly track the (x,y,r) pose of the robot, rather than
 * setting the powers directly from the gamepad inputs. When the translational/rotational components for these poses
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
public class HolonomicVectorDriveTask extends ForeverTask {
    private final Moveable drive;
    private final Supplier<Float> x;
    private final Supplier<Float> y;
    private final Supplier<Float> r;
    private final BooleanSupplier fieldCentricEnabled;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;

    private final ElapsedTime vectorLocker = new ElapsedTime();
    private final ElapsedTime headingLocker = new ElapsedTime();
    private Vector2d vectorLock = null;
    private Double headingLock = null;

    // Default admissible error of 2 inches and 1 degree, waiting 300ms for the pose to stabilise
    private Pose2d toleranceInchRad = new Pose2d(2, 2, Math.toRadians(1));
    private Measure<Time> lockingTimeout = Milliseconds.of(300);

    /**
     * Constructor for HolonomicVectorDriveTask.
     *
     * @param xSupplier           The supplier for the Cartesian x-axis input
     * @param ySupplier           The supplier for the Cartesian y-axis input, <i>note that this will be inverted</i>
     * @param rSupplier           The supplier for the clockwise rotation input
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible. A localizer attached is required.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Supplier<Float> xSupplier, Supplier<Float> ySupplier, Supplier<Float> rSupplier, @NonNull Moveable drive, BooleanSupplier fieldCentricEnabled) {
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        this.drive = drive;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;

        // Sane defaults
        xController = new PController(0.1);
        yController = new PController(0.1);
        rController = new PDController(1, 0.0001);

        withName("Holonomic Vector Control");
    }

    /**
     * Constructor for HolonomicVectorDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver              The gamepad to use for driving
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible. A localizer attached is required.
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Gamepad driver, @NonNull Moveable drive, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, fieldCentricEnabled);
    }

    /**
     * Set the translational PID coefficients. By default it is set to your drive coefficients PID.
     *
     * @param kp proportional
     * @param ki integral
     * @param kd derivative
     * @return this
     */
    public HolonomicVectorDriveTask withTranslationalPID(double kp, double ki, double kd) {
        xController.setPID(kp, ki, kd);
        yController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Set the rotational PID coefficients. By default it is set to your drive coefficients PID.
     *
     * @param kp proportional
     * @param ki integral
     * @param kd derivative
     * @return this
     */
    public HolonomicVectorDriveTask withRotationalPID(double kp, double ki, double kd) {
        rController.setPID(kp, ki, kd);
        return this;
    }

    /**
     * Set the minimum pose error when in self-holding mode to activate correction for.
     *
     * @param inchRad a pose in inches and radians that represents the admissible error for robot auto-correction
     * @return this
     */
    public HolonomicVectorDriveTask withTolerance(Pose2d inchRad) {
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
    public HolonomicVectorDriveTask withStabilisationTimeout(Measure<Time> lockTimeout) {
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
    public HolonomicVectorDriveTask withTolerance(Measure<Distance> poseX, Measure<Distance> poseY, Measure<Angle> poseR) {
        return withTolerance(new Pose2d(poseX.in(Inches), poseY.in(Inches), poseR.in(Radians)));
    }

    /**
     * Set a manual heading that this task should lock to, much as if the robot were rotated to this position
     * and is to try and hold rotation. Note that user input will override this lock, this
     * simply tells the task to respect this value as the locking value.
     *
     * @param heading the angle to rotate to, will be wrapped from [0, 2pi] radians
     */
    public void setHeadingTarget(Measure<Angle> heading) {
        headingLock = Mathf.normaliseAngle(heading).in(Radians);
    }

    /**
     * Set a manual target that this task should lock to, much as if the robot was moved here
     * and is to try and hold position here. Note that user input will override this lock, this
     * simply tells the task to respect this value as the locking value.
     *
     * @param forwardX Forward X component of the locking vector
     * @param strafeY  Strafe Y component of the locking vector
     */
    public void setVectorTarget(Measure<Distance> forwardX, Measure<Distance> strafeY) {
        vectorLock = new Vector2d(forwardX.in(Inches), strafeY.in(Inches));
    }

    @Override
    protected void init() {
        vectorLocker.reset();
        headingLocker.reset();
    }

    @Override
    protected void periodic() {
        Pose2d current = Objects.requireNonNull(drive.getPoseEstimate(), "A localizer must be attached to the drive instance in order to use the HolonomicVectorDriveTask!");

        // Create a new pose based off the user input, which will be the offset from the current pose.
        // Must rotate by 90 degrees (y, -x), then flip y as it is inverted. Rotation must also be inverted as it
        // must be positive anticlockwise.
        double userX = -y.get();
        double userY = -x.get();
        double userR = -r.get();

        double cos = Math.cos(current.heading.toDouble());
        double sin = Math.sin(current.heading.toDouble());

        if (fieldCentricEnabled.getAsBoolean()) {
            // Field-centric inputs that will be rotated before any processing
            double tempX = userX;
            userX = userY * sin + userX * cos;
            userY = userY * cos - tempX * sin;
        }

        // Rising edge detections for pose locking
        if (userX == 0 && userY == 0 && vectorLock == null && vectorLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            vectorLock = current.position;
            // We also reset the controllers as the integral term may be incorrect due to a new target
            xController.reset();
            yController.reset();
        } else if (userX != 0 || userY != 0) {
            vectorLock = null;
            vectorLocker.reset();
        }
        if (userR == 0 && headingLock == null && headingLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            headingLock = current.heading.toDouble();
            rController.reset();
        } else if (userR != 0) {
            headingLock = null;
            headingLocker.reset();
        }

        // Calculate error from current pose to target pose.
        // If we are not locked, the error will be 0, and our error should clamp to zero if it's under the threshold
        double xLockedError = vectorLock == null || Mathf.isNear(vectorLock.x, current.position.x, toleranceInchRad.position.x) ? 0 : vectorLock.x - current.position.x;
        double yLockedError = vectorLock == null || Mathf.isNear(vectorLock.y, current.position.y, toleranceInchRad.position.y) ? 0 : vectorLock.y - current.position.y;
        double rLockedError = headingLock == null || Mathf.isNear(headingLock, current.heading.toDouble(), toleranceInchRad.heading.toDouble()) ? 0 : headingLock - current.heading.toDouble();

        // Rotate error to robot's coordinate frame
        double twistedXError = xLockedError * cos + yLockedError * sin;
        double twistedYError = -xLockedError * sin + yLockedError * cos;

        // Wrap to [-pi, pi] and hard lock at boundary to ensure no oscillations
        double angle = Mathf.inputModulus(rLockedError, -Math.PI, Math.PI);
        if (Mathf.isNear(Math.abs(angle), Math.PI, 0.1))
            angle = -Math.PI * Math.signum(rLockedError);

        drive.setPower(Geometry.poseToVel(new Pose2d(
                vectorLock != null ? -xController.calculate(twistedXError) : userX,
                vectorLock != null ? -yController.calculate(twistedYError) : userY,
                headingLock != null ? -rController.calculate(angle) : userR
        )));

        fieldOverlay.setStroke("#c91c00");
        if (vectorLock != null)
            fieldOverlay.strokeLine(current.position.x, current.position.y, vectorLock.x, vectorLock.y);
        if (headingLock != null)
            Dashboard.drawRobot(fieldOverlay, new Pose2d(current.position, headingLock));
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}

