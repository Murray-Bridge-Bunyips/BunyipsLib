package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;


import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.MecanumDrive.trajectoryIdx;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsLib;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.IMUEx;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TankLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.DriveCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.TankCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.TrajectoryMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MotionProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.TankGains;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * This class is the standard Tank drive class that controls two sets of parallel motors while integrating the
 * RoadRunner v1.0 library in BunyipsLib.
 *
 * @since 6.0.0
 */
public class TankDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final TurnConstraints defaultTurnConstraints;
    private final VelConstraint defaultVelConstraint;
    private final AccelConstraint defaultAccelConstraint;
    private final VoltageSensor voltageSensor;
    private final IMU imu;
    private final List<DcMotorEx> leftMotors, rightMotors;
    private final DriveModel model;
    private final MotionProfile profile;
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", Accumulator.FLIGHT_RECORDER_INTERVAL_MS * 1_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", Accumulator.FLIGHT_RECORDER_INTERVAL_MS * 1_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", Accumulator.FLIGHT_RECORDER_INTERVAL_MS * 1_000_000);
    /**
     * Gains used for tank drive control.
     */
    @NonNull
    public TankGains gains;
    private Localizer localizer;
    private Accumulator accumulator;
    private TankKinematics kinematics;
    private volatile double leftPower;
    private volatile double rightPower;

    /**
     * Create a new TankDrive.
     *
     * @param driveModel           the drive model parameters
     * @param motionProfile        the motion profile parameters
     * @param tankGains            the tank gains parameters
     * @param leftMotors           all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors          all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     * @param imu                  the IMU to use, see {@link IMUEx} for lazy or no hub IMU ({@code IMUEx.none()}) initialisation
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     * @param startPose            the starting pose of the robot
     */
    public TankDrive(@NonNull DriveModel driveModel, @NonNull MotionProfile motionProfile, @NonNull TankGains tankGains, @NonNull List<DcMotor> leftMotors, @NonNull List<DcMotor> rightMotors, @NonNull IMU imu, @NonNull HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping, @NonNull Pose2d startPose) {
        assertParamsNotNull(driveModel, motionProfile, tankGains, leftMotors, rightMotors, imu, voltageSensorMapping, startPose);

        accumulator = new Accumulator(startPose);

        gains = tankGains;
        model = driveModel;
        profile = motionProfile;

        for (DcMotor m : leftMotors) {
            if (assertParamsNotNull(m))
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotor m : rightMotors) {
            if (assertParamsNotNull(m))
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.leftMotors = leftMotors.stream().map(m -> (DcMotorEx) m).collect(Collectors.toList());
        this.rightMotors = rightMotors.stream().map(m -> (DcMotorEx) m).collect(Collectors.toList());
        this.imu = imu;

        withLocalizer(new TankLocalizer(model, leftMotors, rightMotors));
        defaultTurnConstraints = new TurnConstraints(motionProfile.maxAngVel, -motionProfile.maxAngAccel, motionProfile.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(motionProfile.maxWheelVel),
                new AngularVelConstraint(motionProfile.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(motionProfile.minProfileAccel, motionProfile.maxProfileAccel);

        voltageSensor = voltageSensorMapping.iterator().next();

        FlightRecorder.write("RR_TANK_GAINS", gains);
        FlightRecorder.write("RR_MOTION_PROFILE", profile);
    }

    /**
     * Create a new TankDrive that will start at the last known pose.
     *
     * @param driveModel           the drive model parameters
     * @param motionProfile        the motion profile parameters
     * @param tankGains            the tank gains parameters
     * @param leftMotors           all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors          all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     * @param imu                  the IMU to use, see {@link IMUEx} for lazy or null initialisation
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     */
    public TankDrive(@NonNull DriveModel driveModel, @NonNull MotionProfile motionProfile, @NonNull TankGains tankGains, @NonNull List<DcMotor> leftMotors, @NonNull List<DcMotor> rightMotors, @NonNull IMU imu, @NonNull HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping) {
        this(driveModel, motionProfile, tankGains, leftMotors, rightMotors, imu, voltageSensorMapping, Storage.memory().lastKnownPosition);
    }

    /**
     * Create a new TankDrive that will start at the last known pose and use the first found voltage sensor in the Hardware Map.
     *
     * @param driveModel    the drive model parameters
     * @param motionProfile the motion profile parameters
     * @param tankGains     the tank gains parameters
     * @param leftMotors    all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors   all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     * @param imu           the IMU to use, see {@link IMUEx} for lazy or null initialisation
     */
    public TankDrive(@NonNull DriveModel driveModel, @NonNull MotionProfile motionProfile, @NonNull TankGains tankGains, @NonNull List<DcMotor> leftMotors, @NonNull List<DcMotor> rightMotors, @NonNull IMU imu) {
        this(driveModel, motionProfile, tankGains, leftMotors, rightMotors, imu, BunyipsLib.getOpMode().hardwareMap.voltageSensor, Storage.memory().lastKnownPosition);
    }

    /**
     * Set the Localizer this drive instance should use.
     * If not specified, this will be a {@link TankLocalizer}.
     *
     * @param localizer the localizer to use
     * @return this
     */
    @NonNull
    @Override
    public TankDrive withLocalizer(@NonNull Localizer localizer) {
        kinematics = new TankKinematics(model.inPerTick * model.trackWidthTicks);
        FlightRecorder.write("RR_DRIVE_MODEL", model);
        this.localizer = localizer;
        return this;
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @NonNull
    @Override
    public Accumulator getAccumulator() {
        return accumulator;
    }

    /**
     * Set the pose accumulator this drive instance should use.
     * If not specified, the default {@link Accumulator} will be used.
     *
     * @param accumulator the new accumulator to use
     * @return this
     */
    @NonNull
    @Override
    public TankDrive withAccumulator(@NonNull Accumulator accumulator) {
        this.accumulator.copyTo(accumulator);
        this.accumulator = accumulator;
        return this;
    }

    /**
     * Set the raw power for each side of the robot.
     *
     * @param leftPower  the power for the left side of the robot
     * @param rightPower the power for the right side of the robot
     */
    public void setMotorPowers(double leftPower, double rightPower) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    /**
     * Get the raw power for each side of the robot.
     *
     * @return the power for the left and right sides of the robot
     */
    @NonNull
    public double[] getMotorPowers() {
        return new double[]{leftPower, rightPower};
    }

    @Override
    public void setPower(@NonNull PoseVelocity2d target) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2)
                .inverse(PoseVelocity2dDual.constant(target, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftPower = wheelVels.left.get(0) / maxPowerMag;
        rightPower = wheelVels.right.get(0) / maxPowerMag;
    }

    @Override
    public void periodic() {
        accumulator.accumulate(localizer.update());

        Pose2d pose = accumulator.getPose();
        PoseVelocity2d poseVelRot = pose.heading.times(accumulator.getVelocity());
        Telemetry.Item i = DualTelemetry.smartAdd("Localizer", "X:%in(%/s) Y:%in(%/s) %°(%/s)",
                Mathf.round(pose.position.x, 1),
                Mathf.round(poseVelRot.linearVel.x, 1),
                Mathf.round(pose.position.y, 1),
                Mathf.round(poseVelRot.linearVel.y, 1),
                Mathf.round(Math.toDegrees(pose.heading.toDouble()), 1),
                Mathf.round(Math.toDegrees(poseVelRot.angVel), 1)
        );
        if (i instanceof DualTelemetry.HtmlItem hi)
            hi.color("gray").small();

        for (DcMotorEx m : leftMotors) {
            m.setPower(leftPower);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(rightPower);
        }
        tankCommandWriter.write(new TankCommandMessage(voltageSensor.getVoltage(), leftPower, rightPower));

        DualTelemetry.smartAdd(toString(), "L:%\\% %, R:%\\% %",
                Math.round(Math.min(100, Math.abs(leftPower * 100))), leftPower >= 0 ? "↑" : "↓",
                Math.round(Math.min(100, Math.abs(rightPower * 100))), rightPower >= 0 ? "↑" : "↓"
        );
    }

    @Override
    protected void onDisable() {
        leftPower = 0;
        rightPower = 0;
        leftMotors.forEach(m -> m.setPower(0));
        rightMotors.forEach(m -> m.setPower(0));
    }

    @Override
    @NonNull
    public Pose2d getPose() {
        return accumulator.getPose();
    }

    @Override
    public void setPose(@NonNull Pose2d newPose) {
        accumulator.setPose(newPose);
    }

    @Override
    @NonNull
    public PoseVelocity2d getVelocity() {
        return accumulator.getVelocity();
    }

    @NonNull
    @Override
    public Constants getConstants() {
        return new Constants(
                model,
                profile,
                TurnTask::new,
                FollowTrajectoryTask::new,
                new TrajectoryBuilderParams(
                        1.0e-6,
                        new ProfileParams(
                                0.25, 0.1, 1.0e-2
                        )
                ),
                0.0, defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    /**
     * A task that will execute a RoadRunner trajectory on a Tank drive.
     */
    public final class FollowTrajectoryTask extends Task {
        private final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;

        /**
         * Create a new FollowTrajectoryTask.
         *
         * @param t the trajectory to follow
         */
        public FollowTrajectoryTask(@NonNull TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / Dashboard.TRAJECTORY_PREVIEW_SAMPLING_REDUCTION)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
            // Reuse the one from MecanumDrive for simplicity
            FlightRecorder.write("TRAJECTORY_" + trajectoryIdx++, new TrajectoryMessage(xPoints, yPoints));

            timeout(Seconds.of(t.duration));
            named(Text.format("Trajectory %->%",
                    Geometry.toUserString(t.path.begin(1).value()).replace("Pose2d", ""),
                    Geometry.toUserString(t.path.end(1).value())).replace("Pose2d", ""));
            on(TankDrive.this, true);
        }

        @Override
        protected void periodic() {
            DualNum<Time> x = timeTrajectory.profile.get(getElapsedTime().in(Seconds));

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2dDual<Time> feedback = new RamseteController(kinematics.trackWidth, gains.ramseteZeta, gains.ramseteBBar)
                    .compute(x, txWorldTarget, accumulator.getPose());
            driveCommandWriter.write(new DriveCommandMessage(feedback));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
            double voltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftPower = feedforward.compute(wheelVels.left) / voltage;
            rightPower = feedforward.compute(wheelVels.right) / voltage;

            Pose2d error = txWorldTarget.value().minusExp(accumulator.getPose());
            dashboard.put("xError", error.position.x);
            dashboard.put("yError", error.position.y);
            dashboard.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = dashboard.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, txWorldTarget.value());

            c.setStroke("#4CAF50BB");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

        @Override
        protected boolean isTaskFinished() {
            return getElapsedTime().in(Seconds) >= timeTrajectory.duration;
        }

        @Override
        protected void onFinish() {
            leftPower = 0;
            rightPower = 0;
        }
    }

    /**
     * A task that will execute a motion-profiled turn on a Tank drive using RoadRunner.
     */
    public final class TurnTask extends Task {
        private final TimeTurn turn;

        /**
         * Create a new TurnTask.
         *
         * @param turn the turn to execute
         */
        public TurnTask(@NonNull TimeTurn turn) {
            this.turn = turn;
            timeout(Seconds.of(turn.duration));
            named(Text.format("Turn %°->%°",
                    Mathf.round(Math.toDegrees(turn.get(0).value().heading.log()), 1),
                    Mathf.round(Math.toDegrees(turn.get(turn.duration).value().heading.log()), 1)));
            on(TankDrive.this, true);
        }

        @Override
        protected void periodic() {
            Pose2dDual<Time> txWorldTarget = turn.get(getElapsedTime().in(Seconds));
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = accumulator.getVelocity();

            PoseVelocity2dDual<Time> feedback = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(Geometry.zeroVec(), 3),
                    txWorldTarget.heading.velocity().plus(
                            gains.turnGain * accumulator.getPose().heading.minus(txWorldTarget.heading.value()) +
                                    gains.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
            double voltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftPower = feedforward.compute(wheelVels.left) / voltage;
            rightPower = feedforward.compute(wheelVels.right) / voltage;

            Canvas c = dashboard.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, txWorldTarget.value());

            c.setStroke("#7C4DFFBB");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

        @Override
        protected boolean isTaskFinished() {
            return getElapsedTime().in(Seconds) >= turn.duration;
        }

        @Override
        protected void onFinish() {
            leftPower = 0;
            rightPower = 0;
        }
    }
}
