package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Motor;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * RoadRunner implementation for a 2+ wheel Tank drive on REV hardware.
 * <p>
 * This class directly extends the base {@code SampleTankDrive} found in the RoadRunner v0.5 quickstart,
 * and is wrapped to be a {@link BunyipsSubsystem} via {@link TankDrive}. Use the {@link TankDrive} class
 * to use this drive in a {@link BunyipsOpMode}. This class requires you pass in an instance of {@link DriveConstants},
 * which has been adapted to allow multiple configurations for any drive.
 *
 * @since 1.0.0-pre
 */
public class TankRoadRunnerDrive extends com.acmerobotics.roadrunner.drive.TankDrive implements RoadRunnerDrive {
    private final DriveConstants constants;
    private final TankCoefficients coefficients;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private final TrajectoryVelocityConstraint velConstraint;
    private final TrajectoryAccelerationConstraint accelConstraint;

    private final TrajectoryFollower follower;
    private final DualTelemetry telemetry;

    private final List<DcMotorEx> motors;
    private final List<DcMotorEx> leftMotors;
    private final List<DcMotorEx> rightMotors;
    private final IMU imu;

    private final VoltageSensor batteryVoltageSensor;

    /**
     * Create a new TankRoadRunnerDrive with the given parameters. Omits optional {@link DualTelemetry} parameter.
     *
     * @param constants     The drive constants
     * @param coefficients  The tank coefficients
     * @param voltageSensor The voltage sensor mapping ({@code hardwareMap.voltageSensor})
     * @param imu           The IMU for the robot. May be nullable if you are using three-wheel odometry.
     * @param leftMotors    The motors on the left side of the robot (e.g. {@code Arrays.asList(fl, bl)})
     * @param rightMotors   The motors on the right side of the robot (e.g. {@code Arrays.asList(fr, br)})
     */
    public TankRoadRunnerDrive(DriveConstants constants, TankCoefficients coefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, List<DcMotor> leftMotors, List<DcMotor> rightMotors) {
        this(null, constants, coefficients, voltageSensor, imu, leftMotors, rightMotors);
    }

    /**
     * Create a new TankRoadRunnerDrive with the given parameters.
     *
     * @param telemetry     The (optional) DualTelemetry instance to use for telemetry.
     * @param constants     The drive constants
     * @param coefficients  The tank coefficients
     * @param voltageSensor The voltage sensor mapping ({@code hardwareMap.voltageSensor})
     * @param imu           The IMU for the robot. May be nullable if you are using three-wheel odometry.
     * @param leftMotors    The motors on the left side of the robot (e.g. {@code Arrays.asList(fl, bl)})
     * @param rightMotors   The motors on the right side of the robot (e.g. {@code Arrays.asList(fr, br)})
     */
    public TankRoadRunnerDrive(@Nullable DualTelemetry telemetry, DriveConstants constants, TankCoefficients coefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, List<DcMotor> leftMotors, List<DcMotor> rightMotors) {
        super(constants.kV, constants.kA, constants.kStatic, constants.TRACK_WIDTH);

        follower = new TankPIDVAFollower(coefficients.AXIAL_PID, coefficients.CROSS_TRACK_PID,
                constants.admissibleError.first, constants.admissibleError.second);

        velConstraint = getVelocityConstraint(constants.MAX_VEL, constants.MAX_ANG_VEL, constants.TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(constants.MAX_ACCEL);

        batteryVoltageSensor = voltageSensor.iterator().next();

        this.constants = constants;
        this.coefficients = coefficients;
        this.telemetry = telemetry;

        // Assumes IMU is initialised from RobotConfig
        this.imu = imu;

        this.leftMotors = leftMotors.stream()
                .map((m) -> Objects.requireNonNull((DcMotorEx) m))
                .collect(Collectors.toList());
        this.rightMotors = rightMotors.stream()
                .map((m) -> Objects.requireNonNull((DcMotorEx) m))
                .collect(Collectors.toList());
        motors = Stream.concat(this.leftMotors.stream(), this.rightMotors.stream())
                .collect(Collectors.toList());

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (constants.RUN_USING_ENCODER && constants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants.MOTOR_VELO_PID);
        }

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                telemetry, constants.RUN_USING_ENCODER, follower, coefficients.HEADING_PID, batteryVoltageSensor,
                new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), new ArrayList<>()
        );
    }

    /**
     * Get the velocity constraint for the drive.
     *
     * @param maxVel        The maximum velocity in inches per second
     * @param maxAngularVel The maximum angular velocity in radians per second
     * @param trackWidth    The track width in inches
     * @return The velocity constraint
     */
    @Override
    public TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Get the acceleration constraint for the drive.
     *
     * @param maxAccel The maximum acceleration in inches per second squared
     * @return The acceleration constraint
     */
    @Override
    public TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return trajectorySequenceRunner;
    }

    @Override
    public void stop() {
        cancelTrajectory();
        setMotorPowers(0, 0);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    @SuppressWarnings("rawtypes")
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                velConstraint, accelConstraint,
                constants.MAX_ANG_VEL, constants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (constants.RUN_USING_ENCODER && signal != null) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    @Override
    public DriveConstants getConstants() {
        return constants;
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            if (motor instanceof Motor && runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                if (!((Motor) motor).getRunUsingEncoderController().isPresent())
                    throw new EmergencyStop("A configured motor is a Motor instance using RoadRunner and Velocity PID. You need to set your own controller to use via setRunUsingEncoderController() in your config.");
            } else {
                motor.setPIDFCoefficients(runMode, compensatedCoefficients);
            }
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d current = getPoseEstimate();

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = coefficients.VX_WEIGHT * Math.abs(drivePower.getX())
                    + coefficients.OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            drivePower = new Pose2d(
                    coefficients.VX_WEIGHT * drivePower.getX(),
                    0,
                    coefficients.OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        } else {
            // Ensure the y axis is zeroed out.
            drivePower = new Pose2d(drivePower.getX(), 0, drivePower.getHeading());
        }

        if (telemetry != null) {
            telemetry.dashboardFieldOverlay()
                    .setStroke("#751000")
                    .strokeLine(
                            current.getX(), current.getY(),
                            current.getX() + drivePower.getX() * 24, current.getY() + drivePower.getY() * 24
                    );
        }

        setDrivePower(drivePower);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += constants.encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += constants.encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += constants.encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += constants.encoderTicksToInches(rightMotor.getVelocity());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public double[] getMotorPowers() {
        double[] powers = new double[2];
        for (int i = 0; i < 2; i++) {
            powers[0] = leftMotors.get(i).getPower();
            powers[1] = rightMotors.get(i).getPower();
        }
        return powers;
    }

    @Override
    public void setMotorPowers(double... powers) {
        if (powers.length != 2) {
            throw new IllegalArgumentException("Tank drive requires 2 motor powers");
        }
        setMotorPowers(powers[0], powers[1]);
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu != null ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : 0.0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return imu != null ? imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate : 0.0;
    }

    @Override
    public void cancelTrajectory() {
        trajectorySequenceRunner.cancelTrajectory();
    }
}
