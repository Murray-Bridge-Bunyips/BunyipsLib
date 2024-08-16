package org.murraybridgebunyips.bunyipslib.drive;

import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.Watchdog;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Storage;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This is the standard TankDrive class for modern BunyipsLib robots.
 * This is a component for the RoadRunner Tank Drive, integrating RoadRunner and BunyipsLib to be used
 * as a BunyipsSubsystem. As such, this allows for integrated trajectory and pose management.
 * <p>
 * Note: If you'd like to use deadwheel configurations, you can set them yourself by instantiating your own localizer
 * and setting it via {@link #setLocalizer(Localizer)}. Pose estimate data is discarded when switching a localizer.
 * <p>
 * This class has an integrated safety watchdog to ensure that the drive stops and locks if no updates are being
 * supplied for more than 200ms. This is to prevent a runaway robot in case of a software bug or other issues, as
 * the methods attached to this class, such as {@code setSpeedUsingController()} and {@code setWeightedDrivePower()} will
 * propagate instantly on the drive motors. This is the only type of subsystem that has this feature as it wraps
 * around a RoadRunner drive. Other subsystems follow this safety by default as their methods only propagate hardware
 * when the subsystem is updated.
 *
 * @author Lucas Bubner, 2023
 */
public class TankDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final TankRoadRunnerDrive instance;

    private final Watchdog benji;
    private volatile boolean updates;

    /**
     * Create a new TankDrive instance.
     *
     * @param constants    The drive constants
     * @param coefficients The tank coefficients
     * @param imu          The IMU
     * @param frontLeft    The front left motor
     * @param frontRight   The front right motor
     * @param backLeft     The back left motor
     * @param backRight    The back right motor
     */
    public TankDrive(DriveConstants constants, TankCoefficients coefficients, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        assertParamsNotNull(constants, coefficients, imu, frontLeft, frontRight, backLeft, backRight);
        instance = new TankRoadRunnerDrive(opMode.telemetry, constants, coefficients, opMode.hardwareMap.voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        benji = new Watchdog(() -> {
            if (opMode.isStopRequested()) return;
            Dbg.log(getClass(), "Direct drive updates have been disabled as it has been longer than 200ms since the last call to update().");
            updates = false;
            instance.stop();
        }, 100, 200, TimeUnit.MILLISECONDS);
        updatePoseFromMemory();
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return instance.getTrajectorySequenceRunner();
    }

    @Override
    public void stop() {
        instance.stop();
    }

    @Override
    public void waitForIdle() {
        if (isDisabled() || !updates) return;
        instance.waitForIdle();
    }

    @Override
    public DriveConstants getConstants() {
        return instance.getConstants();
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return instance.trajectoryBuilder(startPose);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return instance.trajectoryBuilder(startPose, reversed);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return instance.trajectoryBuilder(startPose, startHeading);
    }

    @Override
    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return instance.trajectorySequenceBuilder(startPose);
    }

    @Override
    public void turnAsync(double angle) {
        if (isDisabled() || !updates) return;
        instance.turnAsync(angle);
    }

    @Override
    public void turn(double angle) {
        if (isDisabled() || !updates) return;
        instance.turn(angle);
    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {
        if (isDisabled() || !updates) return;
        instance.followTrajectoryAsync(trajectory);
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        if (isDisabled() || !updates) return;
        instance.followTrajectory(trajectory);
    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        if (isDisabled() || !updates) return;
        instance.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        if (isDisabled() || !updates) return;
        instance.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public Pose2d getLastError() {
        return instance.getLastError();
    }

    @Override
    protected void periodic() {
        opMode.telemetry.add("Localizer: X:%cm Y:%cm %deg",
                round(Centimeters.convertFrom(instance.getPoseEstimate().getX(), Inches), 1),
                round(Centimeters.convertFrom(instance.getPoseEstimate().getY(), Inches), 1),
                round(Math.toDegrees(instance.getPoseEstimate().getHeading()), 1)).color("gray");

        // Required to ensure that update() is being called before scheduling any motor updates,
        // using a watchdog to ensure that an update is occurring at least every 200ms.
        // Named after goober Benji, or if you don't like that name then you can
        // call it the "Brakes Engagement Necessity Justification Initiative".
        updates = true;
        if (!benji.isRunning())
            benji.start();
        benji.stroke();

        instance.update();
        Storage.memory().lastKnownPosition = instance.getPoseEstimate();
    }

    @Override
    public boolean isBusy() {
        return instance.isBusy();
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        instance.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        instance.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        instance.setPIDFCoefficients(runMode, coefficients);
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        if (isDisabled() || !updates) return;
        instance.setWeightedDrivePower(drivePower);
    }

    @Override
    public List<Double> getWheelPositions() {
        return instance.getWheelPositions();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return instance.getWheelVelocities();
    }

    @Override
    public double[] getMotorPowers() {
        return instance.getMotorPowers();
    }

    @Override
    public void setMotorPowers(double... powers) {
        if (isDisabled() || !updates) return;
        instance.setMotorPowers(powers);
    }

    @Override
    public double getRawExternalHeading() {
        return instance.getRawExternalHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return instance.getExternalHeadingVelocity();
    }

    @Override
    public Localizer getLocalizer() {
        return instance.getLocalizer();
    }

    @Override
    public void setLocalizer(Localizer localizer) {
        instance.setLocalizer(localizer);
    }

    @Override
    public double getExternalHeading() {
        return instance.getExternalHeading();
    }

    @Override
    public void setExternalHeading(double value) {
        instance.setExternalHeading(value);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return instance.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        instance.setPoseEstimate(value);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return instance.getPoseVelocity();
    }

    @Override
    public void updatePoseEstimate() {
        instance.updatePoseEstimate();
    }

    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        if (isDisabled() || !updates) return;
        instance.setDriveSignal(driveSignal);
    }

    @Override
    public void setDrivePower(Pose2d drivePower) {
        if (isDisabled() || !updates) return;
        instance.setDrivePower(drivePower);
    }

    /**
     * Set the speed of the drive using the controller input.
     *
     * @param x The x value of the controller input
     * @param y The y value of the controller input
     * @param r The r value of the controller input
     * @return The TankDrive instance
     */
    public TankDrive setSpeedUsingController(double x, double y, double r) {
        setWeightedDrivePower(Controls.makeRobotPose(x, y, r));
        return this;
    }

    @Override
    public void cancelTrajectory() {
        instance.cancelTrajectory();
    }
}
