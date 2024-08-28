package org.murraybridgebunyips.bunyipslib.drive;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Milliseconds;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.Watchdog;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.SwitchableLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This is the standard MecanumDrive class for modern BunyipsLib robots.
 * This is a component for the RoadRunner Mecanum Drive, integrating RoadRunner and BunyipsLib to be used
 * as a BunyipsSubsystem. As such, this allows for integrated trajectory and pose management, while using
 * features from BunyipsLib including Task and subsystem-based allocations.
 * <p>
 * This class has an integrated safety watchdog to ensure that the drive stops and locks if no updates are being
 * supplied. This is to prevent a runaway robot in case of a software bug or other issues, as
 * the methods attached to this class, such as {@code setSpeedUsingController()} and {@code setWeightedDrivePower()} will
 * propagate instantly on the drive motors. This is the only type of subsystem that has this feature as it wraps
 * around a RoadRunner drive. Other subsystems follow this safety by default as their methods only propagate hardware
 * when the subsystem is updated. This timeout can be changed as it is a constant in {@link RoadRunner}.
 *
 * @author Lucas Bubner, 2023
 */
public class MecanumDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final MecanumRoadRunnerDrive drive;
    private final IMU imu;

    private final Watchdog benji;
    private volatile boolean updates;

    /**
     * Constructor for the MecanumDrive class.
     *
     * @param constants    The drive constants for the robot.
     * @param coefficients The MecanumCoefficients for the drive.
     * @param imu          The IMU for the robot. Can be set to null if you are using three-wheel odometry.
     * @param fl           The front left motor.
     * @param fr           The front right motor.
     * @param bl           The back left motor.
     * @param br           The back right motor.
     */
    public MecanumDrive(DriveConstants constants, MecanumCoefficients coefficients, @Nullable IMU imu, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        assertParamsNotNull(constants, coefficients, imu, fl, fr, bl, br);
        drive = new MecanumRoadRunnerDrive(opMode.telemetry, constants, coefficients, opMode.hardwareMap.voltageSensor, imu, fl, fr, bl, br);
        benji = new Watchdog(() -> {
            if (opMode.isStopRequested()) return;
            Dbg.warn(getClass(), "Stateful drive updates have been disabled as it has been longer than %ms since the last call to update().", RoadRunner.DRIVE_UPDATE_SAFETY_TIMEOUT.in(Milliseconds));
            updates = false;
            drive.stop();
        }, 100, (long) RoadRunner.DRIVE_UPDATE_SAFETY_TIMEOUT.in(Milliseconds), TimeUnit.MILLISECONDS);
        this.imu = imu;
        updatePoseFromMemory();
    }

    /**
     * Call to use the MecanumLocalizer as a backup localizer alongside the current localizer. Note if you are already
     * using a MecanumLocalizer (default), this will duplicate your localizers and there isn't a point of calling this method.
     * This localizer can be switched/tested as part of the SwitchableLocalizer.
     *
     * @return the SwitchableLocalizer
     */
    public SwitchableLocalizer useFallbackLocalizer() {
        Pose2d curr = drive.getPoseEstimate();
        SwitchableLocalizer localizer = new SwitchableLocalizer(
                drive.getLocalizer(),
                new com.acmerobotics.roadrunner.drive.MecanumDrive.MecanumLocalizer(drive, true)
        );
        setLocalizer(localizer);
        drive.setPoseEstimate(curr);
        return localizer;
    }

    /**
     * Call to set a fallback localizer that can be switched/tested to as part of the SwitchableLocalizer.
     *
     * @param fallback the backup localizer
     * @return the SwitchableLocalizer
     */
    public SwitchableLocalizer useFallbackLocalizer(Localizer fallback) {
        Pose2d curr = drive.getPoseEstimate();
        SwitchableLocalizer localizer = new SwitchableLocalizer(drive.getLocalizer(), fallback);
        setLocalizer(localizer);
        drive.setPoseEstimate(curr);
        return localizer;
    }

    /**
     * Reset the IMU's yaw to 0.
     */
    public void resetYaw() {
        imu.resetYaw();
        setExternalHeading(0);
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return drive.getTrajectorySequenceRunner();
    }

    /**
     * Cleanup and store the last pose estimate in global storage, while stopping the motors.
     */
    @Override
    public void stop() {
        // Safety stop to prevent a runaway robot
        drive.stop();
    }

    public void waitForIdle() {
        if (isDisabled() || !updates) return;
        drive.waitForIdle();
    }

    @Override
    public DriveConstants getConstants() {
        return drive.getConstants();
    }

    public MecanumCoefficients getCoefficients() {
        return drive.getCoefficients();
    }

    /**
     * For continuity, keep setSpeedUsingController for setting drive speeds.
     * Internally runs setWeightedDrivePower() and converts the controller input to a robot Pose2d.
     *
     * @param x gamepad.left_stick_x or similar
     * @param y gamepad.left_stick_y or similar
     * @param r gamepad.right_stick_x or similar
     * @return this
     */
    public MecanumDrive setSpeedUsingController(double x, double y, double r) {
        setWeightedDrivePower(Controls.makeRobotPose(x, y, r));
        return this;
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("Localizer: X:%cm Y:%cm %deg",
                round(Centimeters.convertFrom(drive.getPoseEstimate().getX(), Inches), 1),
                round(Centimeters.convertFrom(drive.getPoseEstimate().getY(), Inches), 1),
                round(Math.toDegrees(drive.getPoseEstimate().getHeading()), 1)).color("gray");

        // Required to ensure that update() is being called before scheduling any motor updates,
        // using a watchdog to ensure that an update is occurring at least every 200ms.
        // Named after goober Benji, or if you don't like that name then you can call
        // it the "Brakes Engagement Necessity Justification Initiative".
        updates = true;
        if (!benji.isRunning())
            benji.start();
        benji.stroke();

        drive.update();
        Storage.memory().lastKnownPosition = drive.getPoseEstimate();
    }

    @Override
    public double getExternalHeading() {
        return drive.getExternalHeading();
    }

    @Override
    public void setExternalHeading(double value) {
        drive.setExternalHeading(value);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    @Override
    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    @Override
    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    @Override
    public void turn(double angle) {
        if (isDisabled() || !updates) return;
        drive.turn(angle);
    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        if (isDisabled() || !updates) return;
        drive.followTrajectory(trajectory);
    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        if (isDisabled() || !updates) return;
        drive.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public Pose2d getLastError() {
        return drive.getLastError();
    }

    @Override
    public boolean isBusy() {
        return drive.isBusy();
    }

    @Override
    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        drive.setPoseEstimate(value);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    @Override
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        if (isDisabled() || !updates) return;
        drive.setDriveSignal(driveSignal);
    }

    @Override
    public void setDrivePower(Pose2d drivePower) {
        if (isDisabled() || !updates) return;
        drive.setDrivePower(drivePower);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        drive.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        drive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(runMode, coefficients);
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        if (isDisabled() || !updates) return;
        drive.setWeightedDrivePower(drivePower);
    }

    @Override
    public List<Double> getWheelPositions() {
        return drive.getWheelPositions();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    @Override
    public double[] getMotorPowers() {
        return drive.getMotorPowers();
    }

    @Override
    public void setMotorPowers(double... powers) {
        if (isDisabled() || !updates) return;
        drive.setMotorPowers(powers);
    }

    @Override
    public double getRawExternalHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @Override
    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    @Override
    public void setLocalizer(Localizer localizer) {
        drive.setLocalizer(localizer);
    }

    @Override
    public void cancelTrajectory() {
        drive.cancelTrajectory();
    }
}