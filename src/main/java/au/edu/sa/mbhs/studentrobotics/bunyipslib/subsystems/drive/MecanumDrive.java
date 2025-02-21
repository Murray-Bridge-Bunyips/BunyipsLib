package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.InchesPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.RadiansPerSecond;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
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
import com.acmerobotics.roadrunner.Rotation2dDual;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.IMUEx;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.MecanumLocalizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.RoadRunnerDrive;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.DriveCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.MecanumCommandMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.DriveModel;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.ErrorThresholds;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MecanumGains;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.MotionProfile;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * This class is the standard Mecanum drive class that controls a set of four mecanum wheels while
 * integrating the RoadRunner v1.0 library in BunyipsLib.
 *
 * @since 6.0.0
 */
public class MecanumDrive extends BunyipsSubsystem implements RoadRunnerDrive {
    /**
     * The lookahead distance used for the current implementation of the path following mode of this MecanumDrive.
     */
    public static double PATH_FOLLOWER_PROJECTION_LOOKAHEAD_INCHES = 6.0;
    /**
     * The minimum distance from endpoint of the trajectory to the projected pose to begin stabilisation for the
     * path following mode of this MecanumDrive.
     */
    public static double PATH_FOLLOWER_ENDPOINT_PROJECTION_TOLERANCE_INCHES = 1.0;

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
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
    /**
     * Gains used for holonomic drive control.
     */
    @NonNull
    public MecanumGains gains;
    private Localizer localizer;
    private Accumulator accumulator;
    private MecanumKinematics kinematics;
    private ErrorThresholds errorThresholds = ErrorThresholds.DEFAULT;
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
     * @param imu                  the IMU to use, see {@link IMUEx} for lazy initialisation
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     * @param startPose            the starting pose of the robot
     */
    public MecanumDrive(@NonNull DriveModel driveModel, @NonNull MotionProfile motionProfile, @NonNull MecanumGains mecanumGains, @Nullable DcMotor leftFront, @Nullable DcMotor leftBack, @Nullable DcMotor rightBack, @Nullable DcMotor rightFront, @Nullable IMU imu, @NonNull HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping, @NonNull Pose2d startPose) {
        accumulator = new Accumulator(startPose);

        gains = mecanumGains;
        model = driveModel;
        profile = motionProfile;

        kinematics = new MecanumKinematics(driveModel.inPerTick * driveModel.trackWidthTicks, driveModel.inPerTick / driveModel.lateralInPerTick);
        defaultTurnConstraints = new TurnConstraints(motionProfile.maxAngVel, -motionProfile.maxAngAccel, motionProfile.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(motionProfile.maxWheelVel),
                new AngularVelConstraint(motionProfile.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(motionProfile.minProfileAccel, motionProfile.maxProfileAccel);

        voltageSensor = voltageSensorMapping.iterator().next();

        FlightRecorder.write("MECANUM_GAINS", mecanumGains);
        FlightRecorder.write("MECANUM_PROFILE", motionProfile);

        if (assertParamsNotNull(driveModel, motionProfile, mecanumGains, leftFront, leftBack, rightBack, rightFront, imu, voltageSensorMapping, startPose)) {
            assert leftFront != null && leftBack != null && rightBack != null && rightFront != null;
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.leftFront = (DcMotorEx) leftFront;
        this.leftBack = (DcMotorEx) leftBack;
        this.rightBack = (DcMotorEx) rightBack;
        this.rightFront = (DcMotorEx) rightFront;
        this.imu = imu;

        if (gains.poseHoldingEnabled) {
            setDefaultTask(new HoldLastPoseTask());
        }

        Dashboard.enableConfig(getClass());
    }

    /**
     * Create a new MecanumDrive that will start at the last known pose.
     *
     * @param driveModel           the drive model parameters
     * @param motionProfile        the motion profile parameters
     * @param mecanumGains         the mecanum gains parameters
     * @param leftFront            the front left motor
     * @param leftBack             the back left motor
     * @param rightBack            the back right motor
     * @param rightFront           the front right motor
     * @param imu                  the IMU to use, see {@link IMUEx} for lazy initialisation
     * @param voltageSensorMapping the voltage sensor mapping for the robot as returned by {@code hardwareMap.voltageSensor}
     */
    public MecanumDrive(@NonNull DriveModel driveModel, @NonNull MotionProfile motionProfile, @NonNull MecanumGains mecanumGains, @Nullable DcMotor leftFront, @Nullable DcMotor leftBack, @Nullable DcMotor rightBack, @Nullable DcMotor rightFront, @Nullable IMU imu, @NonNull HardwareMap.DeviceMapping<VoltageSensor> voltageSensorMapping) {
        this(driveModel, motionProfile, mecanumGains, leftFront, leftBack, rightBack, rightFront, imu, voltageSensorMapping, Storage.memory().lastKnownPosition);
    }

    /**
     * Set the Localizer this drive instance should use.
     * If not specified, this will be a {@link MecanumLocalizer}.
     *
     * @param localizer the localizer to use
     * @return this
     */
    @NonNull
    @Override
    public MecanumDrive withLocalizer(@NonNull Localizer localizer) {
        // Reassign kinematics as it may have been updated
        kinematics = new MecanumKinematics(model.inPerTick * model.trackWidthTicks, model.inPerTick / model.lateralInPerTick);
        FlightRecorder.write("MECANUM_DRIVE_MODEL", model);
        this.localizer = localizer;
        return this;
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        if (localizer == null) {
            localizer = new MecanumLocalizer(model, leftFront, leftBack, rightBack, rightFront, imu);
        }
        return localizer;
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
    public MecanumDrive withAccumulator(@NonNull Accumulator accumulator) {
        this.accumulator.copyTo(accumulator);
        this.accumulator = accumulator;
        return this;
    }

    @NonNull
    @Override
    public Accumulator getAccumulator() {
        return accumulator;
    }

    /**
     * Set a new set of error thresholds for this drive instance to use when running trajectories.
     *
     * @param errorThresholds the new error thresholds to use
     * @return this
     */
    @NonNull
    public MecanumDrive withErrorThresholds(@NonNull ErrorThresholds errorThresholds) {
        this.errorThresholds = errorThresholds;
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
    }

    /**
     * Get the raw power of each motor on the drive.
     *
     * @return the power of each motor on the drive, in the order of left front, left back, right back, right front
     */
    @NonNull
    public double[] getMotorPowers() {
        return new double[]{leftFrontPower, leftBackPower, rightBackPower, rightFrontPower};
    }

    @Override
    public void setPower(@NonNull PoseVelocity2d target) {
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
    public Pose2d getPose() {
        return accumulator.getPose();
    }

    /**
     * Reset the accumulated pose estimate to this pose. Further updates from an internal localizer will continue
     * from this supplied pose estimate.
     *
     * @param newPose the new pose to continue localization from
     */
    @Override
    public void setPose(@NonNull Pose2d newPose) {
        accumulator.setPose(newPose);
    }

    /**
     * Calculate the first derivative of the accumulated pose from an internal localizer.
     *
     * @return the current pose velocity of the drive as of the last update
     */
    @Override
    @NonNull
    public PoseVelocity2d getVelocity() {
        return accumulator.getVelocity();
    }

    /**
     * This method will dispatch new motor powers and run one iteration of the localizer to update the pose estimates.
     * It will also update the pose history for drawing on the dashboard.
     */
    @Override
    public void periodic() {
        if (localizer == null) {
            withLocalizer(new MecanumLocalizer(model, leftFront, leftBack, rightBack, rightFront, imu));
        }

        accumulator.accumulate(localizer.update());

        Pose2d pose = accumulator.getPose();
        PoseVelocity2d poseVel = accumulator.getVelocity();
        Telemetry.Item i = DualTelemetry.smartAdd("Localizer", "X:%in(%/s) Y:%in(%/s) %°(%/s)",
                Mathf.round(pose.position.x, 1),
                Mathf.round(poseVel.linearVel.x, 1),
                Mathf.round(pose.position.y, 1),
                Mathf.round(poseVel.linearVel.y, 1),
                Mathf.round(Math.toDegrees(pose.heading.toDouble()), 1),
                Mathf.round(Math.toDegrees(poseVel.angVel), 1)
        );
        if (i instanceof DualTelemetry.HtmlItem hi)
            hi.color("gray").small();

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        DualTelemetry.smartAdd(toString(), "FL:%\\% %, BL:%\\% %, BR:%\\% %, FR:%\\% %",
                Math.round(Math.min(100, Math.abs(leftFrontPower * 100))), leftFrontPower >= 0 ? "↑" : "↓",
                Math.round(Math.min(100, Math.abs(leftBackPower * 100))), leftBackPower >= 0 ? "↑" : "↓",
                Math.round(Math.min(100, Math.abs(rightBackPower * 100))), rightBackPower >= 0 ? "↑" : "↓",
                Math.round(Math.min(100, Math.abs(rightFrontPower * 100))), rightFrontPower >= 0 ? "↑" : "↓"
        );
    }

    @Override
    protected void onDisable() {
        leftFrontPower = 0;
        leftBackPower = 0;
        rightBackPower = 0;
        rightFrontPower = 0;
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    @NonNull
    @Override
    public Constants getConstants() {
        return new Constants(
                model,
                profile,
                TurnTask::new,
                gains.pathFollowingEnabled ? FollowDisplacementTrajectoryTask::new : FollowTimedTrajectoryTask::new,
                new TrajectoryBuilderParams(
                        1.0e-6,
                        new ProfileParams(
                                0.25, 0.1, 1.0e-2
                        )
                ),
                0, defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    /**
     * A task that will hold the last known pose of the robot using the RoadRunner feedback loop.
     * Useful as a default task and can be conveniently autoconfigured as the default task via the {@link MecanumGains} config option.
     */
    public final class HoldLastPoseTask extends Task {
        private Pose2d hold;

        @Override
        protected void init() {
            hold = accumulator.getPose();
            named("Hold " + Geometry.toUserString(hold).replace("Pose2d", ""));
        }

        @Override
        protected void periodic() {
            targetPoseWriter.write(new PoseMessage(hold));

            Pose2d robotPose = accumulator.getPose();
            PoseVelocity2d robotVel = accumulator.getVelocity();
            // Target position with zero velocity and acceleration
            Pose2dDual<Time> txWorldTarget = new Pose2dDual<>(
                    Vector2dDual.constant(hold.position, 3),
                    Rotation2dDual.constant(hold.heading, 3)
            );

            PoseVelocity2dDual<Time> feedback = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, robotPose, robotVel);
            driveCommandWriter.write(new DriveCommandMessage(feedback));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
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

            Pose2d error = hold.minusExp(robotPose);
            dashboard.put("xError", error.position.x);
            dashboard.put("yError", error.position.y);
            dashboard.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = dashboard.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, hold);

            c.setStroke("#4CAF50BB");
            c.strokeLine(robotPose.position.x, robotPose.position.y, hold.position.x, hold.position.y);
        }
    }

    /**
     * A task that will execute a RoadRunner trajectory on a Mecanum drive using a path follower approach.
     * Can be enabled for use via the {@link MecanumGains} configuration option.
     */
    public final class FollowDisplacementTrajectoryTask extends Task {
        private final DisplacementTrajectory displacementTrajectory;
        private final double[] xPoints, yPoints;
        private Pose2d error = Geometry.zeroPose();
        private ElapsedTime stab;
        private PoseVelocity2d robotVelRobot = Geometry.zeroVel();
        private double s;

        /**
         * Create a new FollowDisplacementTrajectoryTask.
         *
         * @param t the trajectory to follow with a path follower
         */
        public FollowDisplacementTrajectoryTask(@NonNull TimeTrajectory t) {
            displacementTrajectory = new DisplacementTrajectory(t.path, t.profile.dispProfile);
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

            named(Text.format("Trajectory %::%",
                    Geometry.toUserString(t.path.begin(1).value()).replace("Pose2d", ""),
                    Geometry.toUserString(t.path.end(1).value())).replace("Pose2d", ""));
            on(MecanumDrive.this, true);
        }

        @Override
        protected void periodic() {
            Pose2d actualPose = accumulator.getPose();
            robotVelRobot = accumulator.getVelocity();

            s = displacementTrajectory.project(actualPose.position, s);
            // Internally reparameterises from respect to displacement to respect to time
            // Future: currently using a static lookahead, should be improved in the future to something more dynamic
            Pose2dDual<Time> txWorldTarget = displacementTrajectory.get(s + PATH_FOLLOWER_PROJECTION_LOOKAHEAD_INCHES);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2dDual<Time> feedback = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, actualPose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(feedback));

            double voltage = voltageSensor.getVoltage();

            // Future: calculate max feedforward power
//            Pose2dDual<Arclength> displacement = displacementTrajectory.path.get(s, 3);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
            MotorFeedforward feedforward = new MotorFeedforward(profile.kS,
                    profile.kV / model.inPerTick, profile.kA / model.inPerTick);
            leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            error = txWorldTarget.value().minusExp(actualPose);
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
            boolean displacement = s + PATH_FOLLOWER_ENDPOINT_PROJECTION_TOLERANCE_INCHES >= displacementTrajectory.length();
            if (displacement && stab == null) {
                stab = new ElapsedTime();
            } else if (!displacement) {
                stab = null;
            }
            boolean stablilised = stab != null &&
                    ((error.position.norm() < errorThresholds.getMaxTranslationalError().in(Inches)
                            && robotVelRobot.linearVel.norm() < errorThresholds.getMinVelStab().in(InchesPerSecond)
                            && error.heading.toDouble() < errorThresholds.getMaxAngularError().in(Radians)
                            && robotVelRobot.angVel < errorThresholds.getMinAngVelStab().in(RadiansPerSecond))
                            || stab.seconds() > errorThresholds.getStabilizationTimeout().in(Seconds));
            return displacement && stablilised;
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
     * A task that will execute a time-based RoadRunner trajectory on a Mecanum drive.
     */
    public final class FollowTimedTrajectoryTask extends Task {
        private final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private Pose2d error = Geometry.zeroPose();
        private PoseVelocity2d robotVelRobot = Geometry.zeroVel();
        private double beginTs = -1;
        private double t;

        /**
         * Create a new FollowTimedTrajectoryTask.
         *
         * @param t the trajectory to follow with a time-based approach (default)
         */
        public FollowTimedTrajectoryTask(@NonNull TimeTrajectory t) {
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

            timeout(Seconds.of(t.duration).plus(errorThresholds.getStabilizationTimeout()));
            named(Text.format("Trajectory %->%",
                    Geometry.toUserString(t.path.begin(1).value()).replace("Pose2d", ""),
                    Geometry.toUserString(t.path.end(1).value())).replace("Pose2d", ""));
            on(MecanumDrive.this, true);
        }

        @Override
        protected void periodic() {
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2d robotPose = accumulator.getPose();
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            robotVelRobot = accumulator.getVelocity();

            PoseVelocity2dDual<Time> feedback = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, robotPose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(feedback));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
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

            error = txWorldTarget.value().minusExp(robotPose);
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
            return t >= timeTrajectory.duration
                    && error.position.norm() < errorThresholds.getMaxTranslationalError().in(Inches)
                    && robotVelRobot.linearVel.norm() < errorThresholds.getMinVelStab().in(InchesPerSecond)
                    && error.heading.toDouble() < errorThresholds.getMaxAngularError().in(Radians)
                    && robotVelRobot.angVel < errorThresholds.getMinAngVelStab().in(RadiansPerSecond);
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
        private Pose2d error = Geometry.zeroPose();
        private PoseVelocity2d robotVelRobot = Geometry.zeroVel();
        private double beginTs = -1;
        private double t;

        /**
         * Create a new TurnTask.
         *
         * @param turn the turn to execute
         */
        public TurnTask(@NonNull TimeTurn turn) {
            this.turn = turn;
            timeout(Seconds.of(turn.duration).plus(errorThresholds.getStabilizationTimeout()));
            named(Text.format("Turn %°->%°",
                    Mathf.round(Math.toDegrees(turn.get(0).value().heading.log()), 1),
                    Mathf.round(Math.toDegrees(turn.get(turn.duration).value().heading.log()), 1)));
            on(MecanumDrive.this, true);
        }

        @Override
        protected void periodic() {
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            robotVelRobot = accumulator.getVelocity();

            PoseVelocity2dDual<Time> feedback = new HolonomicController(
                    gains.axialGain, gains.lateralGain, gains.headingGain,
                    gains.axialVelGain, gains.lateralVelGain, gains.headingVelGain
            )
                    .compute(txWorldTarget, accumulator.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(feedback));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(feedback);
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

            error = txWorldTarget.value().minusExp(accumulator.getPose());
            dashboard.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = dashboard.fieldOverlay();
            c.setStrokeWidth(1);

            c.setStroke("#4CAF50");
            Dashboard.drawRobot(c, txWorldTarget.value());

            c.setStroke("#7C4DFFBB");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }

        @Override
        protected boolean isTaskFinished() {
            return t >= turn.duration
                    && error.heading.toDouble() < errorThresholds.getMaxAngularError().in(Radians)
                    && robotVelRobot.angVel < errorThresholds.getMinAngVelStab().in(RadiansPerSecond);
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
