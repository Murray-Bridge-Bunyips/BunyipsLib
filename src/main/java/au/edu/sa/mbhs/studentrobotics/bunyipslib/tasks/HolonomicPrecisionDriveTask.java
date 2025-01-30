package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.DegreesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.DegreesPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecondPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MotionProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import kotlin.NotImplementedError;

/**
 * Gamepad drive for all holonomic drivetrains which will use a velocity vector to control the robot.
 * This task is designed to be used as a default/standard-priority task, other tasks will override it.
 * <p>
 * Compared to {@link HolonomicDriveTask}, this task will constantly track the (x,y,r) velocity of the robot.
 * Kinematic constraints translates user input into a target velocity, generating a target vector for the robot
 * relative to the frame. Using a set of system controllers, this target velocity is attempted to be reached, while
 * obeying the constraints of the original trapezoidal profile.
 * <p>
 * This mode allows precise control over movement of the robot, where the velocity as desired by the vector of the gamepad
 * will be reflected on the robot with the assistance of high-accuracy odometry. This mode is comparable to a more
 * kinematically corrective and advanced version of the {@link HolonomicTrackingDriveTask}.
 * <p>
 * A localizer-attached holonomic drive is required for this class, as it will require the use of the pose estimate system.
 *
 * @author Lucas Bubner, 2025
 * @since 7.0.0
 */
public class HolonomicPrecisionDriveTask extends FieldOrientableDriveTask {
    /**
     * Default controller to use for the x (forward) velocity axis.
     */
    @NonNull
    public static SystemController DEFAULT_X_CONTROLLER = new PController(8);
    /**
     * Default controller to use for the y (strafe) velocity axis.
     */
    @NonNull
    public static SystemController DEFAULT_Y_CONTROLLER = new PController(8);
    /**
     * Default controller to use for the r (rotation) velocity axis.
     */
    @NonNull
    public static SystemController DEFAULT_R_CONTROLLER = new PController(12);

    private final Supplier<PoseVelocity2d> vel;
    private SystemController xController;
    private SystemController yController;
    private SystemController rController;
    // Sane defaults with unbounded acceleration
    private Measure<Velocity<Distance>> maxVel = InchesPerSecond.of(40);
    private Measure<Velocity<Velocity<Distance>>> maxAccel = InchesPerSecondPerSecond.of(Double.MAX_VALUE);
    private Measure<Velocity<Angle>> maxAngVel = DegreesPerSecond.of(180);
    private Measure<Velocity<Velocity<Angle>>> maxAngAccel = DegreesPerSecondPerSecond.of(Double.MAX_VALUE);
    // TODO: do we need to use a locking vector technique?
    // TODO: trapezoidal profile (consider profiling regardless instead of relying fully on the pids?)

    /**
     * Constructor for HolonomicPrecisionDriveTask.
     *
     * @param targetVecNormalised The supplier for the current pose velocity of the robot via the controller as a fraction [-1, 1] of the maximum velocity
     * @param drive               The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *                            called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *                            if possible. A localizer attached is required. A {@link RoadRunnerDrive} will auto-extract maximum <i>velocity</i> constraints.
     */
    public HolonomicPrecisionDriveTask(@NonNull Supplier<PoseVelocity2d> targetVecNormalised, @NonNull Moveable drive) {
        super.drive = drive;
        if (drive instanceof BunyipsSubsystem s)
            on(s, false);
        if (drive instanceof RoadRunnerDrive rrd) {
            MotionProfile mp = rrd.getConstants().getMotionProfile();
            // Only extract velocity, if the user wants acceleration also we leave it to them
            maxVel = InchesPerSecond.of(mp.maxWheelVel);
            maxAngVel = RadiansPerSecond.of(mp.maxAngVel);
        }
        vel = targetVecNormalised;

        withXController(DEFAULT_X_CONTROLLER);
        withYController(DEFAULT_Y_CONTROLLER);
        withRController(DEFAULT_R_CONTROLLER);

        named("Holonomic Precision Control");
    }

    /**
     * Constructor for HolonomicPrecisionDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver The gamepad to use for driving
     * @param drive  The holonomic drive to use, which you must ensure is holonomic as strafe commands will be
     *               called unlike the differential control task. This task will be auto-attached to this BunyipsSubsystem
     *               if possible. A localizer attached is required.
     */
    public HolonomicPrecisionDriveTask(@NonNull Gamepad driver, @NonNull Moveable drive) {
        this(() -> Controls.vel(driver.left_stick_x, driver.left_stick_y, driver.right_stick_x), drive);
    }

    /**
     * Sets the velocity controller for the x (forward) axis.
     *
     * @param x the controller to use
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withXController(@NonNull SystemController x) {
        xController = x;
        return this;
    }

    /**
     * Sets the velocity controller for the y (strafe) axis.
     *
     * @param y the controller to use
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withYController(@NonNull SystemController y) {
        yController = y;
        return this;
    }

    /**
     * Sets the velocity controller for the r (rotation) axis.
     *
     * @param r the controller to use
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withRController(@NonNull SystemController r) {
        rController = r;
        return this;
    }

    /**
     * Sets the maximum velocity of your robot that can be used as a target for your gamepad input.
     *
     * @param maxVel the maximum velocity; note for {@link RoadRunnerDrive} instances the maximum velocity fields
     *               are extracted for you in the constructor, and not calling this method will auto-set the constraints to this task.
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withMaxVel(Measure<Velocity<Distance>> maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    /**
     * Sets the maximum anglular velocity of your robot that can be used as a target for your gamepad input.
     *
     * @param maxAngVel the maximum angular velocity; note for {@link RoadRunnerDrive} instances the maximum angular velocity fields
     *                  are extracted for you in the constructor, and not calling this method will auto-set the constraints to this task.
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withMaxAngVel(Measure<Velocity<Angle>> maxAngVel) {
        this.maxAngVel = maxAngVel;
        return this;
    }

    /**
     * Sets the maximum acceleration of your robot that will be respected during velocity setting of your robot.
     * This value is by default is set to {@link Double#MAX_VALUE} to disable acceleration control. Setting a new
     * acceleration will cause the velocity inputs to follow this profile.
     *
     * @param maxAccel the maximum acceleration; note {@link RoadRunnerDrive} instances <b>DO NOT</b> extract this field automatically
     *                 as acceleration constraints should be handled only through this method to enable the limits
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withMaxAccel(Measure<Velocity<Velocity<Distance>>> maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    /**
     * Sets the maximum angular acceleration of your robot that will be respected during velocity setting of your robot.
     * This value is by default is set to {@link Double#MAX_VALUE} to disable angular acceleration control. Setting a new
     * acceleration will cause the angular velocity inputs to follow this profile.
     *
     * @param maxAngAccel the maximum angular acceleration; note {@link RoadRunnerDrive} instances <b>DO NOT</b> extract this field automatically
     *                    as acceleration constraints should be handled only through this method to enable the limits
     * @return this
     */
    @NonNull
    public HolonomicPrecisionDriveTask withMaxAngAccel(Measure<Velocity<Velocity<Angle>>> maxAngAccel) {
        this.maxAngAccel = maxAngAccel;
        return this;
    }

    @Override
    protected void periodic() {
        PoseVelocity2d input = applyOrientation(vel.get());
        PoseVelocity2d currentVelocity = Objects.requireNonNull(drive.getVelocity(), "A drive localizer able to supply velocity information is required to use the HolonomicPrecisionDriveTask!");
        // TODO: do we need to implement a circular scaling to ensure the maxVel is followed for 2d motion?
        double xVel = xController.calculate(currentVelocity.linearVel.x, maxVel.in(InchesPerSecond) * input.linearVel.x);
        double yVel = yController.calculate(currentVelocity.linearVel.y, maxVel.in(InchesPerSecond) * input.linearVel.y);
        double rVel = rController.calculate(currentVelocity.angVel, maxAngVel.in(RadiansPerSecond) * input.angVel);
        // TODO: this approach is unstable, need to think of a better way to do this
        drive.setPower(Geometry.vel(xVel, yVel, rVel));
        throw new NotImplementedError("hpdt not implemented yet!");
    }

    @Override
    protected void onFinish() {
        drive.setPower(Geometry.zeroVel());
    }
}

