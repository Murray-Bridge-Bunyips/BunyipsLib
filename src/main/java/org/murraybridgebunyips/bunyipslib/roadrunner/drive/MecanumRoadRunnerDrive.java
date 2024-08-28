package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * RoadRunner implementation for a 4-wheel Mecanum drive on REV hardware.
 * <p>
 * This class directly extends the base {@code SampleMecanumDrive} found in the RoadRunner v0.5 quickstart,
 * and is wrapped to be a {@link BunyipsSubsystem} via {@link MecanumDrive}. Use the {@link MecanumDrive} class
 * to use this drive in a {@link BunyipsOpMode}. This class requires you pass in an instance of {@link DriveConstants},
 * which has been adapted to allow multiple configurations for any drive.
 *
 * @since 1.0.0-pre
 */
public class MecanumRoadRunnerDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements RoadRunnerDrive {
    private final DriveConstants constants;
    private final TrajectoryVelocityConstraint velConstraint;
    private final TrajectoryAccelerationConstraint accelConstraint;
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryFollower follower;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;
    private final IMU imu;
    private final VoltageSensor batteryVoltageSensor;
    private final List<Integer> lastEncPositions = new ArrayList<>();
    private final List<Integer> lastEncVels = new ArrayList<>();
    private MecanumCoefficients coefficients;

    /**
     * Constructor for the MecanumRoadRunnerDrive class. Omits optional {@link DualTelemetry} parameter.
     *
     * @param constants     The drive constants for the robot.
     * @param coefficients  The coefficients for the mecanum drive.
     * @param voltageSensor The voltage sensor mapping ({@code hardwareMap.voltageSensor})
     * @param imu           The IMU for the robot. May be nullable if you are using three-wheel odometry.
     * @param fl            The front left motor.
     * @param fr            The front right motor.
     * @param bl            The back left motor.
     * @param br            The back right motor.
     */
    public MecanumRoadRunnerDrive(DriveConstants constants, MecanumCoefficients coefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, @Nullable IMU imu, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this(null, constants, coefficients, voltageSensor, imu, fl, fr, bl, br);
    }

    /**
     * Constructor for the MecanumRoadRunnerDrive class.
     *
     * @param telemetry     The (optional) DualTelemetry instance to use for telemetry.
     * @param constants     The drive constants for the robot.
     * @param coefficients  The coefficients for the mecanum drive.
     * @param voltageSensor The voltage sensor for the robot from hardwareMap.
     * @param imu           The IMU for the robot. May be nullable if you are using three-wheel odometry.
     * @param fl            The front left motor.
     * @param fr            The front right motor.
     * @param bl            The back left motor.
     * @param br            The back right motor.
     */
    public MecanumRoadRunnerDrive(@Nullable DualTelemetry telemetry, DriveConstants constants, MecanumCoefficients coefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, @Nullable IMU imu, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        super(constants.kV, constants.kA, constants.kStatic, constants.TRACK_WIDTH, constants.TRACK_WIDTH, coefficients.LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(coefficients.TRANSLATIONAL_PID, coefficients.TRANSLATIONAL_PID, coefficients.HEADING_PID,
                constants.admissibleError.first, constants.admissibleError.second);

        velConstraint = getVelocityConstraint(constants.MAX_VEL, constants.MAX_ANG_VEL, constants.TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(constants.MAX_ACCEL);

        batteryVoltageSensor = voltageSensor.iterator().next();

        this.constants = constants;
        this.coefficients = coefficients;

        assert fl != null && fr != null && bl != null && br != null && imu != null;

        // Assumes IMU was initialised from RobotConfig
        this.imu = imu;

        leftFront = (DcMotorEx) fl;
        leftRear = (DcMotorEx) bl;
        rightRear = (DcMotorEx) br;
        rightFront = (DcMotorEx) fr;

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (constants.RUN_USING_ENCODER && constants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, constants.MOTOR_VELO_PID);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                telemetry, constants.RUN_USING_ENCODER, follower, coefficients.HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public MecanumCoefficients getCoefficients() {
        return coefficients;
    }

    public void setCoefficients(MecanumCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public DriveConstants getConstants() {
        return constants;
    }

    @Override
    public TrajectorySequenceRunner getTrajectorySequenceRunner() {
        return trajectorySequenceRunner;
    }

    @Override
    public void stop() {
        cancelTrajectory();
        setMotorPowers(0, 0, 0, 0);
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

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
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
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = coefficients.VX_WEIGHT * Math.abs(drivePower.getX())
                    + coefficients.VY_WEIGHT * Math.abs(drivePower.getY())
                    + coefficients.OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    coefficients.VX_WEIGHT * drivePower.getX(),
                    coefficients.VY_WEIGHT * drivePower.getY(),
                    coefficients.OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(constants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(constants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    public double[] getMotorPowers() {
        double[] powers = new double[4];
        for (int i = 0; i < 4; i++) {
            powers[i] = motors.get(i).getPower();
        }
        return powers;
    }

    @Override
    public void setMotorPowers(double... powers) {
        if (powers.length != 4) {
            throw new IllegalArgumentException("Invalid number of powers passed");
        }
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
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