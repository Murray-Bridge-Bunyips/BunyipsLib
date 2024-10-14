package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Drawing;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.MecanumLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.MecanumGains;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.MotionProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.DriveCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.MecanumCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * This class is the standard Mecanum drive class that controls a set of four mecanum wheels while
 * integrating the RoadRunner v1.0 library in BunyipsLib.
 *
 * @since 6.0.0
 */
public class MecanumDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    private final MecanumKinematics kinematics;
    private final TurnConstraints defaultTurnConstraints;
    private final VelConstraint defaultVelConstraint;
    private final AccelConstraint defaultAccelConstraint;

    private final VoltageSensor voltageSensor;
    private final IMU imu;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;

    private final DriveModel model;
    private final MotionProfile profile;
    private final MecanumGains gains;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

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
    private volatile double leftFrontPower;
    private volatile double leftBackPower;
    private volatile double rightBackPower;
    private volatile double rightFrontPower;

    /**
     * Create a new MecanumDrive.
     *
     * @param driveModel           the drive model parameters
     * @param motionProfile        the motion profile parameters
     * @param mecanumGains         the mecanum gains parameters
     * @param leftFront            the front left motor
     * @param leftBack             the back left motor
     * @param rightBack            the back right motor
     * @param rightFront           the front right motor
     * @param imu                  the IMU for the robot, see DynIMU for flexible IMU usage
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     * @param startPose            the starting pose of the robot
     */
    public MecanumDrive(DriveModel driveModel, MotionProfile motionProfile, MecanumGains mecanumGains, DcMotor leftFront, DcMotor leftBack, DcMotor rightBack, DcMotor rightFront, IMU imu, HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping, Pose2d startPose) {
        pose = startPose;
        gains = mecanumGains;
        model = driveModel;
        profile = motionProfile;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftFront = (DcMotorEx) leftFront;
        this.leftBack = (DcMotorEx) leftBack;
        this.rightBack = (DcMotorEx) rightBack;
        this.rightFront = (DcMotorEx) rightFront;
        this.imu = imu;

        kinematics = new MecanumKinematics(driveModel.inPerTick * driveModel.trackWidthTicks, driveModel.inPerTick / driveModel.lateralInPerTick);
        defaultTurnConstraints = new TurnConstraints(motionProfile.maxAngVel, -motionProfile.maxAngAccel, motionProfile.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(motionProfile.maxWheelVel),
                new AngularVelConstraint(motionProfile.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(motionProfile.minProfileAccel, motionProfile.maxProfileAccel);

        voltageSensor = voltageSensorMapping.iterator().next();
        localizer = new MecanumLocalizer(driveModel, leftFront, leftBack, rightBack, rightFront, imu);

        FlightRecorder.write("MECANUM_GAINS", mecanumGains);
        FlightRecorder.write("MECANUM_DRIVE_MODEL", driveModel);
        FlightRecorder.write("MECANUM_PROFILE", motionProfile);
    }

    /**
     * Set the Localizer this drive instance should use.
     * If not specified, this will be a {@link MecanumLocalizer}.
     *
     * @param localizer the localizer to use
     * @return this
     */
    public MecanumDrive withLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    /**
     * Set the raw power of each motor on the drive.
     *
     * @param leftFrontPower  the power of the front left motor
     * @param leftBackPower   the power of the back left motor
     * @param rightBackPower  the power of the back right motor
     * @param rightFrontPower the power of the front right motor
     */
    public void setMotorPowers(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        this.leftFrontPower = leftFrontPower;
        this.leftBackPower = leftBackPower;
        this.rightBackPower = rightBackPower;
        this.rightFrontPower = rightFrontPower;

        // TODO: SimpleMecanumDrive and SimpleTankDrive
        // TODO: RoadRunner unit-based builder (like old RoadRunner util)
        // TODO: RoadRunnerTuningOpMode (and new tuning OpModes)
    }

    /**
     * Get the raw power of each motor on the drive.
     *
     * @return the power of each motor on the drive, in the order of left front, left back, right back, right front
     */
    public double[] getMotorPowers() {
        return new double[]{leftFrontPower, leftBackPower, rightBackPower, rightFrontPower};
    }

    @Override
    public void setPower(PoseVelocity2d target) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1)
                .inverse(PoseVelocity2dDual.constant(target, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFrontPower = wheelVels.leftFront.get(0) / maxPowerMag;
        leftBackPower = wheelVels.leftBack.get(0) / maxPowerMag;
        rightBackPower = wheelVels.rightBack.get(0) / maxPowerMag;
        rightFrontPower = wheelVels.rightFront.get(0) / maxPowerMag;
    }

    /**
     * Calculate the accumulated pose from an internal localizer.
     *
     * @return the current pose estimate of the drive
     */
    @Override
    @NonNull
    public Pose2d getPoseEstimate() {
        return pose;
    }

    /**
     * Reset the accumulated pose estimate to this pose. Further updates from an internal localizer will continue
     * from this supplied pose estimate.
     *
     * @param newPose the new pose to continue localization from
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d newPose) {
        pose = newPose;
    }

    /**
     * Calculate the first derivative of the accumulated pose from an internal localizer.
     *
     * @return the current pose velocity of the drive as of the last update
     */
    @Override
    @NonNull
    public PoseVelocity2d getPoseVelocity() {
        return poseVelo;
    }

    /**
     * This method will dispatch new motor powers and run one iteration of the localizer to update the pose estimates.
     * It will also update the pose history for drawing on the dashboard.
     */
    @Override
    protected void periodic() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());
        poseVelo = twist.velocity().value();

        poseHistory.add(pose);
        while (poseHistory.size() > Drawing.MAX_POSE_HISTORY) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(pose));

        Drawing.useCanvas(c -> {
            c.setStrokeWidth(1);
            c.setStroke("#3F51B5");
            Drawing.drawPoseHistory(c, poseHistory);
            Drawing.drawRobot(c, pose);

            // TODO: test
            Vector2d directionOfTravel = pose.heading
                    .times(twist.value().line)
                    // 24 for 1 field tile in inches
                    .times(24);
            c.setStroke("#751000")
                    .strokeLine(
                            pose.position.x,
                            pose.position.y,
                            pose.position.x + directionOfTravel.x,
                            pose.position.y + directionOfTravel.y
                    );
        });

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    @Override
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnTask::new,
                FollowTrajectoryTask::new,
                new TrajectoryBuilderParams(
                        1.0e-6,
                        new ProfileParams(
                                0.25, 0.1, 1.0e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    @Override
    public TrajectoryActionFactory newTrajectoryTask() {
        return FollowTrajectoryTask::new;
    }

    @Override
    public TurnActionFactory newTurnTask() {
        return TurnTask::new;
    }

    @Override
    public DriveModel getDriveModel() {
        return model;
    }

    @Override
    public MotionProfile getMotionProfile() {
        return profile;
    }

    /**
     * A task that will execute a RoadRunner trajectory on a Mecanum drive.
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

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = getPoseVelocity();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = p.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

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
            leftFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            rightFrontPower = 0;
        }
    }

    /**
     * A task that will execute a motion-profiled turn on a Mecanum drive using RoadRunner.
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

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            MotorFeedforward feedforward = new MotorFeedforward(
                    profile.kS, profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            Canvas c = p.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

        @Override
        protected boolean isTaskFinished() {
            return t >= turn.duration;
        }

        @Override
        protected void onFinish() {
            leftFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            rightFrontPower = 0;
        }
    }
}
