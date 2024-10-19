package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;


import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Actions;
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
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.RobotConfig;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.TankLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.DriveCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.TankCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MotionProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.TankGains;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * This class is the standard Tank drive class that controls two sets of parallel motors while integrating the
 * RoadRunner v1.0 library in BunyipsLib.
 *
 * @since 6.0.0
 */
public class TankDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final TankKinematics kinematics;
    private final TurnConstraints defaultTurnConstraints;
    private final VelConstraint defaultVelConstraint;
    private final AccelConstraint defaultAccelConstraint;

    private final VoltageSensor voltageSensor;
    private final LazyImu lazyImu;
    private final List<DcMotorEx> leftMotors, rightMotors;

    private final DriveModel model;
    private final MotionProfile profile;
    private final TankGains gains;

    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    // Exposed as public for ease of use and to match the quickstart, will also expose poseVelo for completeness,
    // although poseVelo cannot be reassigned by the user and must instead be done via the power setter.
    /**
     * The current pose estimate of the robot which may be modified to reflect a new pose.
     */
    public Pose2d pose;
    /**
     * (Read-only) The current pose velocity of the robot.
     */
    public PoseVelocity2d poseVelo;

    private Localizer localizer;
    private Accumulator accumulator;
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
     * @param lazyImu              the LazyImu instance to use, see the {@code getLazyImu} method of {@link RobotConfig} to construct this
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     * @param startPose            the starting pose of the robot
     */
    public TankDrive(DriveModel driveModel, MotionProfile motionProfile, TankGains tankGains, List<DcMotor> leftMotors, List<DcMotor> rightMotors, LazyImu lazyImu, HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping, Pose2d startPose) {
        assertParamsNotNull(driveModel, motionProfile, tankGains, leftMotors, rightMotors, lazyImu, voltageSensorMapping, startPose);

        accumulator = new Accumulator(startPose);
        Storage.memory().lastKnownPosition = startPose;
        pose = startPose;

        gains = tankGains;
        model = driveModel;
        profile = motionProfile;

        for (DcMotor m : leftMotors) {
            assertParamsNotNull(m);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotor m : rightMotors) {
            assertParamsNotNull(m);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.leftMotors = leftMotors.stream().map(m -> (DcMotorEx) m).collect(Collectors.toList());
        this.rightMotors = rightMotors.stream().map(m -> (DcMotorEx) m).collect(Collectors.toList());
        this.lazyImu = lazyImu;

        kinematics = new TankKinematics(driveModel.inPerTick * driveModel.trackWidthTicks);
        defaultTurnConstraints = new TurnConstraints(motionProfile.maxAngVel, -motionProfile.maxAngAccel, motionProfile.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(motionProfile.maxWheelVel),
                new AngularVelConstraint(motionProfile.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(motionProfile.minProfileAccel, motionProfile.maxProfileAccel);

        voltageSensor = voltageSensorMapping.iterator().next();
        localizer = new TankLocalizer(model, leftMotors, rightMotors);

        FlightRecorder.write("TANK_GAINS", gains);
        FlightRecorder.write("TANK_DRIVE_MODEL", model);
        FlightRecorder.write("TANK_PROFILE", profile);
    }

    /**
     * Create a new TankDrive that will start at the last known pose.
     *
     * @param driveModel           the drive model parameters
     * @param motionProfile        the motion profile parameters
     * @param tankGains            the tank gains parameters
     * @param leftMotors           all motors on the left side of the robot (e.g. {@code Arrays.asList(leftFront, leftBack)})
     * @param rightMotors          all motors on the right side of the robot (e.g. {@code Arrays.asList(rightFront, rightBack)})
     * @param lazyImu              the LazyImu instance to use, see the {@code getLazyImu} method of {@link RobotConfig} to construct this
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     */
    public TankDrive(DriveModel driveModel, MotionProfile motionProfile, TankGains tankGains, List<DcMotor> leftMotors, List<DcMotor> rightMotors, LazyImu lazyImu, HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping) {
        this(driveModel, motionProfile, tankGains, leftMotors, rightMotors, lazyImu, voltageSensorMapping, Storage.memory().lastKnownPosition);
    }

    /**
     * Set the Localizer this drive instance should use.
     * If not specified, this will be a {@link TankLocalizer}.
     *
     * @param localizer the localizer to use
     * @return this
     */
    public TankDrive withLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     * Set the pose accumulator this drive instance should use.
     * If not specified, the default {@link Accumulator} will be used.
     *
     * @param accumulator the new accumulator to use
     * @return this
     */
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

        leftPower = Mathf.clamp(wheelVels.left.get(0) / maxPowerMag, -1, 1);
        rightPower = Mathf.clamp(wheelVels.right.get(0) / maxPowerMag, -1, 1);
    }

    @Override
    public void periodic() {
        Twist2dDual<Time> twist = localizer.update();
        accumulator.setPoseEstimate(pose);
        accumulator.accumulate(twist);
        pose = accumulator.getPoseEstimate();
        poseVelo = accumulator.getPoseVelocity();

        opMode(o -> o.telemetry.add("Localizer: X:%in(%/s) Y:%in(%/s) %deg(%/s)",
                Mathf.round(pose.position.x, 1),
                Mathf.round(poseVelo.linearVel.x, 1),
                Mathf.round(pose.position.y, 1),
                Mathf.round(poseVelo.linearVel.y, 1),
                Mathf.round(Math.toDegrees(pose.heading.toDouble()), 1),
                Mathf.round(Math.toDegrees(poseVelo.angVel), 1)
        ).color("gray"));

        for (DcMotorEx m : leftMotors) {
            m.setPower(leftPower);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(rightPower);
        }

        opMode(o -> o.telemetry.add("%: L:%\\% %, R:%\\% %", this,
                Math.round(leftPower * 100), leftPower >= 0 ? "↑" : "↓",
                Math.round(rightPower * 100), rightPower >= 0 ? "↑" : "↓"
        ));
    }

    @Override
    protected void onDisable() {
        leftMotors.forEach(m -> m.setPower(0));
        rightMotors.forEach(m -> m.setPower(0));
    }

    @Override
    @NonNull
    public Pose2d getPoseEstimate() {
        return pose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d newPose) {
        pose = newPose;
    }

    @Override
    @NonNull
    public PoseVelocity2d getPoseVelocity() {
        return poseVelo;
    }

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
        private double beginTs = -1;
        private double t;

        /**
         * Create a new FollowTrajectoryTask.
         *
         * @param t the trajectory to follow
         */
        public FollowTrajectoryTask(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }

            withTimeout(Seconds.of(t.duration));
        }

        @Override
        protected void periodic() {
            fieldOverlay.setStroke("#4CAF507A");
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.strokePolyline(xPoints, yPoints);

            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, gains.ramseteZeta, gains.ramseteBBar)
                    .compute(x, txWorldTarget, pose);
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftPower = feedforward.compute(wheelVels.left) / voltage;
            rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = p.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, txWorldTarget.value());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }

        @Override
        protected boolean isTaskFinished() {
            return t >= timeTrajectory.duration;
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
        private double beginTs = -1;
        private double t;

        /**
         * Create a new TurnTask.
         *
         * @param turn the turn to execute
         */
        public TurnTask(TimeTurn turn) {
            this.turn = turn;
            withTimeout(Seconds.of(turn.duration));
        }

        @Override
        protected void periodic() {
            fieldOverlay.setStroke("#7C4DFF7A");
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = getPoseVelocity();

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(Geometry.zeroVec(), 3),
                    txWorldTarget.heading.velocity().plus(
                            gains.turnGain * pose.heading.minus(txWorldTarget.heading.value()) +
                                    gains.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftPower = feedforward.compute(wheelVels.left) / voltage;
            rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            Canvas c = p.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, txWorldTarget.value());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

        @Override
        protected boolean isTaskFinished() {
            return t >= turn.duration;
        }

        @Override
        protected void onFinish() {
            leftPower = 0;
            rightPower = 0;
        }
    }
}
