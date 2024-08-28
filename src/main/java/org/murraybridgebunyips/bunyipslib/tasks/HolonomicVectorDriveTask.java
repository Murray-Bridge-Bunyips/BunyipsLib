package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Nanoseconds;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
 * A RoadRunner drive is required for this class, as it will require the use of the pose estimate system and other
 * coefficients such as your PID. Therefore, the only supported class this task will work for is {@link MecanumDrive}.
 *
 * @author Lucas Bubner, 2024
 * @since 4.0.0
 */
public class HolonomicVectorDriveTask extends ForeverTask {
    private final MecanumDrive drive;
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
     * @param mecanumDrive        The MecanumDrive to use
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Supplier<Float> xSupplier, Supplier<Float> ySupplier, Supplier<Float> rSupplier, @NotNull MecanumDrive mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        onSubsystem(mecanumDrive, false);
        drive = mecanumDrive;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;

        PIDCoefficients translationCoeffs = drive.getCoefficients().TRANSLATIONAL_PID;
        PIDCoefficients rotationCoeffs = drive.getCoefficients().HEADING_PID;
        xController = new PIDController(translationCoeffs.kP, translationCoeffs.kI, translationCoeffs.kD);
        yController = new PIDController(translationCoeffs.kP, translationCoeffs.kI, translationCoeffs.kD);
        rController = new PIDController(rotationCoeffs.kP, rotationCoeffs.kI, rotationCoeffs.kD);

        withName("Holonomic Vector Control");
    }

    /**
     * Constructor for HolonomicVectorDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver              The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Gamepad driver, @NotNull MecanumDrive mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, mecanumDrive, fieldCentricEnabled);
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
        Pose2d current = drive.getPoseEstimate();

        // Create a new pose based off the user input, which will be the offset from the current pose.
        // Must rotate by 90 degrees (y, -x), then flip y as it is inverted. Rotation must also be inverted as it
        // must be positive anticlockwise.
        double userX = -y.get();
        double userY = -x.get();
        double userR = -r.get();

        double cos = Math.cos(current.getHeading());
        double sin = Math.sin(current.getHeading());

        if (fieldCentricEnabled.getAsBoolean()) {
            // Field-centric inputs that will be rotated before any processing
            double tempX = userX;
            userX = userY * sin + userX * cos;
            userY = userY * cos - tempX * sin;
        }

        // Rising edge detections for pose locking
        if (userX == 0 && userY == 0 && vectorLock == null && vectorLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            vectorLock = current.vec();
            // We also reset the controllers as the integral term may be incorrect due to a new target
            xController.reset();
            yController.reset();
        } else if (userX != 0 || userY != 0) {
            vectorLock = null;
            vectorLocker.reset();
        }
        if (userR == 0 && headingLock == null && headingLocker.nanoseconds() >= lockingTimeout.in(Nanoseconds)) {
            headingLock = current.getHeading();
            rController.reset();
        } else if (userR != 0) {
            headingLock = null;
            headingLocker.reset();
        }

        // Calculate error from current pose to target pose.
        // If we are not locked, the error will be 0, and our error should clamp to zero if it's under the threshold
        double xLockedError = vectorLock == null || Mathf.isNear(vectorLock.getX(), current.getX(), toleranceInchRad.getX()) ? 0 : vectorLock.getX() - current.getX();
        double yLockedError = vectorLock == null || Mathf.isNear(vectorLock.getY(), current.getY(), toleranceInchRad.getY()) ? 0 : vectorLock.getY() - current.getY();
        double rLockedError = headingLock == null || Mathf.isNear(headingLock, current.getHeading(), toleranceInchRad.getHeading()) ? 0 : headingLock - current.getHeading();

        // Rotate error to robot's coordinate frame
        double twistedXError = xLockedError * cos + yLockedError * sin;
        double twistedYError = -xLockedError * sin + yLockedError * cos;

        // Wrap to [-pi, pi] and hard lock at boundary to ensure no oscillations
        double angle = Mathf.inputModulus(rLockedError, -Math.PI, Math.PI);
        if (Mathf.isNear(Math.abs(angle), Math.PI, 0.1))
            angle = -Math.PI * Math.signum(rLockedError);

        drive.setWeightedDrivePower(
                new Pose2d(
                        vectorLock != null ? -xController.calculate(twistedXError) : userX,
                        vectorLock != null ? -yController.calculate(twistedYError) : userY,
                        headingLock != null ? -rController.calculate(angle) : userR
                )
        );
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }
}

