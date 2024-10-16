package org.murraybridgebunyips.bunyipslib.vision;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Filter;
import org.murraybridgebunyips.bunyipslib.drive.Moveable;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Objects;
import java.util.function.Predicate;

/**
 * Combines an AprilTag processor and RoadRunner drive to supply updates in pose estimation.
 *
 * @author Lucas Bubner, 2024
 * @since 3.2.0
 */
@Config
public class AprilTagPoseEstimator implements Runnable {
    /**
     * Heading stabilisation extra low-pass gain.
     */
    public static double HEADING_STAB_LP_GAIN = 0.98;
    /**
     * Default R gain for Kalman filtering.
     */
    public static double DEFAULT_R = 4;
    /**
     * Default Q gain for Kalman filtering.
     */
    public static double DEFAULT_Q = 1.0e-4;

    private final AprilTag processor;
    private final Localizer localizer;
    private final HashSet<Predicate<AprilTagData>> filters = new HashSet<>();
    private final ArrayList<Pose2d> estimates = new ArrayList<>();

    private final Filter.LowPass headingStab = new Filter.LowPass(HEADING_STAB_LP_GAIN);
    private Filter.Kalman xf;
    private Filter.Kalman yf;
    private Filter.Kalman rf;

    private Pose2d previousOffset = new Pose2d();
    private Pose2d cameraRobotOffset = new Pose2d();
    private boolean active = true;
    private boolean updateHeading = true;

    /**
     * Constructor for AprilTagPoseEstimator.
     * <p>
     * Note that the option to also update heading based off these readings is enabled by default.
     * Disable with {@link #setHeadingEstimate}.
     * <p>
     * Also note that by default, this pose estimator will assume the camera is exactly at the center of the robot, facing forward.
     * Adjust this using {@link #setCameraOffset}.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param drive     the drive localizer to use for pose information
     */
    public AprilTagPoseEstimator(AprilTag processor, Moveable drive) {
        this(processor, Objects.requireNonNull(drive.getLocalizer(), "Non-null localizer required to be attached to the drive instance"));
    }

    /**
     * Constructor for AprilTagPoseEstimator.
     * <p>
     * Note that the option to also update heading based off these readings is enabled by default.
     * Disable with {@link #setHeadingEstimate}.
     * <p>
     * Also note that by default, this pose estimator will assume the camera is exactly at the center of the robot, facing forward.
     * Adjust this using {@link #setCameraOffset}.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param localizer the localizer to use for pose information
     */
    public AprilTagPoseEstimator(AprilTag processor, @NonNull Localizer localizer) {
        this.processor = processor;
        this.localizer = localizer;
        setKalmanGains(DEFAULT_R, DEFAULT_Q);

        BunyipsOpMode.ifRunning(opMode -> {
            opMode.onActiveLoop(this);
            Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode.");
        });
    }

    /**
     * Create a new AprilTagPoseEstimator runner.
     * <p>
     * Note that the option to also update heading based off these readings is enabled by default.
     * Disable with {@link #setHeadingEstimate}.
     * <p>
     * Also note that by default, this pose estimator will assume the camera is exactly at the center of the robot, facing forward.
     * Adjust this using {@link #setCameraOffset}.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param localizer the localizer to use for pose information
     * @return a new AprilTagPoseEstimator instance
     */
    public static AprilTagPoseEstimator enable(AprilTag processor, @NonNull Localizer localizer) {
        return new AprilTagPoseEstimator(processor, localizer);
    }

    /**
     * Create a new AprilTagPoseEstimator runner.
     * <p>
     * Note that the option to also update heading based off these readings is enabled by default.
     * Disable with {@link #setHeadingEstimate}.
     * <p>
     * Also note that by default, this pose estimator will assume the camera is exactly at the center of the robot, facing forward.
     * Adjust this using {@link #setCameraOffset}.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param drive     the drive localizer to use for pose information
     * @return a new AprilTagPoseEstimator instance
     */
    public static AprilTagPoseEstimator enable(AprilTag processor, Moveable drive) {
        return new AprilTagPoseEstimator(processor, drive);
    }

    /**
     * Set the gains used in the Kalman filter for the x, y, and r components.
     *
     * @param R higher values of R will put more trust in the odometry
     * @param Q higher values of Q trusts the AprilTag more
     * @return this
     * @since 4.1.0
     */
    public AprilTagPoseEstimator setKalmanGains(double R, double Q) {
        xf = new Filter.Kalman(R, Q);
        yf = new Filter.Kalman(R, Q);
        rf = new Filter.Kalman(R, Q);
        return this;
    }

    /**
     * Add a data filter that will apply to the detections of the processor whether to accept processing this tag.
     *
     * @param filter the filter to add
     * @return this
     * @since 5.1.0
     */
    public AprilTagPoseEstimator addDataFilter(Predicate<AprilTagData> filter) {
        filters.add(filter);
        return this;
    }

    /**
     * Remove a filter instance added in {@link #addDataFilter(Predicate)}
     *
     * @param filter the filter to remove
     * @return this
     * @since 5.1.0
     */
    public AprilTagPoseEstimator removeDataFilter(Predicate<AprilTagData> filter) {
        if (!filters.remove(filter))
            Dbg.warn(getClass(), "Unable to remove filter '%', not found.", filter);
        return this;
    }

    /**
     * Whether to set the drive pose to the vision estimate. Default is on.
     *
     * @param setPoseAutomatically whether AprilTagPoseEstimator is on
     * @return this
     */
    public AprilTagPoseEstimator setActive(boolean setPoseAutomatically) {
        active = setPoseAutomatically;
        return this;
    }

    /**
     * Whether to set the drive pose to the vision estimate with heading information. Default is ON.
     *
     * @param setHeadingAutomatically whether to also set the heading to the AprilTag estimate or not
     * @return this
     */
    public AprilTagPoseEstimator setHeadingEstimate(boolean setHeadingAutomatically) {
        updateHeading = setHeadingAutomatically;
        return this;
    }

    /**
     * Set a robot-relative offset of the webcam from the center of your robot, facing towards your defined robot's forward.
     * This offset will be used to generate a more accurate pose estimate, as camera rotation and translation on a robot
     * will impact the accuracy of the pose estimator.
     * <p>
     * If you do not call this method, your camera will be assumed to be at the center of the robot, facing forward.
     * Note that the rotation offset is only used when paired with the heading estimator (see {@link #setHeadingEstimate}).
     *
     * @param forwardFromCenter the forward (Cartesian Y) distance from the center of the robot to the camera, extending outwards FORWARD looking top down with your robot facing forward
     * @param leftFromCenter    the lateral (Cartesian X) distance from the center of the robot to the camera, extending outwards LEFT looking top down with your robot facing forward
     * @param rotationOnRobot   the angle at which, increasing ANTI-CLOCKWISE, your camera is pointing relative to your robot facing forward
     * @return this
     */
    public AprilTagPoseEstimator setCameraOffset(Measure<Distance> forwardFromCenter, Measure<Distance> leftFromCenter, Measure<Angle> rotationOnRobot) {
        // Convert to native units
        cameraRobotOffset = new Pose2d(
                forwardFromCenter.in(Inches),
                leftFromCenter.in(Inches),
                rotationOnRobot.in(Radians)
        );
        return this;
    }

    /**
     * Set a robot-relative offset of the webcam from the center of your robot, facing towards your defined robot's forward.
     * This offset will be used to generate a more accurate pose estimate, as camera rotation and translation on a robot
     * will impact the accuracy of the pose estimator.
     * <p>
     * If you do not call this method, your camera will be assumed to be at the center of the robot, facing forward.
     * Note that the rotation offset is only used when paired with the heading estimator (see {@link #setHeadingEstimate}).
     *
     * @param poseInchRad an inches vector and radians pose conforming to the requirements as met by {@link #setCameraOffset(Measure, Measure, Measure)}, listed below.<br>
     *                    <b>Pose X</b>: the forward (Cartesian Y) distance (inches) from the center of the robot to the camera, extending outwards FORWARD looking top down with your robot facing forward<br>
     *                    <b>Pose Y</b>: the lateral (Cartesian X) distance (inches) from the center of the robot to the camera, extending outwards LEFT looking top down with your robot facing forward<br>
     *                    <b>Pose Heading</b>: the angle (radians) at which, increasing ANTI-CLOCKWISE, your camera is pointing relative to your robot facing forward
     * @return this
     * @see #setCameraOffset(Measure, Measure, Measure)
     */
    public AprilTagPoseEstimator setCameraOffset(Pose2d poseInchRad) {
        // Assumes native units are already present
        cameraRobotOffset = poseInchRad;
        return this;
    }

    /**
     * Propagate interpretation of AprilTag processor and set the pose estimate to the tag, if
     * available by the SDK. This method will no-op if insufficient information is available.
     */
    @Override
    public void run() {
        if (!active || !processor.isRunning())
            return;

        Pose2d poseEstimate = (localizer.getPoseEstimate()).minus(previousOffset);
        ArrayList<AprilTagData> data = processor.getData();
        if (data.isEmpty())
            return;

        estimates.clear();
        for (int i = 0; i < data.size(); i++) {
            AprilTagData aprilTag = data.get(i);
            if (!aprilTag.isInLibrary() || filters.stream().anyMatch(f -> !f.test(aprilTag))) {
                // No luck with this ID
                continue;
            }

            assert aprilTag.getMetadata().isPresent() && aprilTag.getFtcPose().isPresent();
            AprilTagMetadata metadata = aprilTag.getMetadata().get();
            AprilTagPoseFtc camPose = aprilTag.getFtcPose().get();

            VectorF tagPos = metadata.fieldPosition;
            Orientation tagOri = metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            double tagX = tagPos.get(0);
            double tagY = tagPos.get(1);
            double tagRotation = metadata.distanceUnit.toInches(tagOri.thirdAngle);
            // 2D transformation matrix
            // x' = x * cos(t) - y * sin(t)
            // y' = x * sin(t) + y * cos(t)
            // where t=0 yields (-y, x) for a 90 degree default rotation to accommodate for the 90 degree offset
            // between RoadRunner pose and the FTC Global Coordinate system.
            double t = tagRotation - Math.toRadians(camPose.yaw);
            double relativeX = camPose.x * Math.cos(t) - camPose.y * Math.sin(t);
            double relativeY = camPose.x * Math.sin(t) + camPose.y * Math.cos(t);
            // Displacement vector
            Vector2d pos = new Vector2d(
                    tagX - relativeX,
                    tagY - relativeY
            );
            // Offset as defined by the user to account for the camera not representing true position
            pos = pos.minus(cameraRobotOffset.vec()
                    .rotated(tagRotation + Math.PI / 2)
                    .rotated(-Math.toRadians(camPose.yaw)));

            // Only set the heading if the user wants it, which we can do fairly simply if they want that too
            double heading = poseEstimate.getHeading();
            if (updateHeading) {
                // Rotate 90 degrees (pi/2 rads) to match unit circle proportions due to a rotation mismatch as corrected
                // in the vector calculation above
                heading = Math.PI / 2.0 + tagRotation - Math.toRadians(camPose.yaw) - cameraRobotOffset.getHeading();
            }
            Pose2d atPoseEstimate = new Pose2d(pos.getX(), pos.getY(), Mathf.inputModulus(heading, -Math.PI, Math.PI));

            // Apply Kalman filters to the vector components
            Pose2d kfPose = new Pose2d(
                    xf.calculate(poseEstimate.getX(), atPoseEstimate.getX()),
                    yf.calculate(poseEstimate.getY(), atPoseEstimate.getY()),
                    updateHeading ? rf.calculate(
                            Mathf.inputModulus(poseEstimate.getHeading(), -Math.PI, Math.PI),
                            Mathf.inputModulus(atPoseEstimate.getHeading(), -Math.PI, Math.PI)
                    ) : poseEstimate.getHeading()
            );

            // Avoid spamming the logs by logging the events that are over an inch away from the current estimate
            if (atPoseEstimate.vec().distTo(kfPose.vec()) >= 1)
                Dbg.logd(getClass(), "AprilTag ID#% seen, %,%->%", aprilTag.getId(), poseEstimate, atPoseEstimate, kfPose);

            estimates.add(kfPose);
        }

        // Use the average pose calculated by each tag
        Pose2d avg = estimates.stream()
                .reduce(new Pose2d(), Pose2d::plus)
                .div(estimates.size());

        // Use an unmodified pose as the one we actually calculate otherwise we'll oscillate around the target
        // since the Kalman filter shouldn't be fed it's own data
        previousOffset = avg.minus(poseEstimate);
        previousOffset = new Pose2d(previousOffset.vec(),
                headingStab.apply(Mathf.inputModulus(previousOffset.getHeading(), 0, 2 * Math.PI)));

        localizer.setPoseEstimate(avg);
    }
}
