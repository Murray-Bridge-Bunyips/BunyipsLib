package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.Predicate;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Filter;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

/**
 * Combines an {@link AprilTag} processor with an {@link Accumulator} to relocalize the robot.
 * This serves as the modernised version of the previous {@code AprilTagPoseEstimator}.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class AprilTagRelocalizingAccumulator extends Accumulator {
    /**
     * Default R gain for Kalman filtering.
     */
    public static double DEFAULT_R = 4;
    /**
     * Default Q gain for Kalman filtering.
     */
    public static double DEFAULT_Q = 0.01;

    private final AprilTag processor;
    private final HashSet<Predicate<AprilTagData>> filters = new HashSet<>();
    private final ArrayList<Pose2d> estimates = new ArrayList<>();

    private Filter.Kalman xf;
    private Filter.Kalman yf;
    private Filter.Kalman rf;

    private double lastHeading;
    private boolean active = true;
    private boolean updateHeading = false;
    private boolean useKf = true;

    /**
     * Create a new AprilTag relocalizing accumulator.
     *
     * @param processor the processor to use for relocalization, assumes that the camera pose property on the builder
     *                  has been set (see {@link AprilTag#setCameraPose(AprilTagProcessor.Builder)} for a util)
     */
    public AprilTagRelocalizingAccumulator(@NonNull AprilTag processor) {
        this.processor = processor;
        setKalmanGains(DEFAULT_R, DEFAULT_Q);
        Dashboard.enableConfig(getClass());
    }

    /**
     * Set the gains used in the Kalman filter for the x, y, and r components.
     *
     * @param R higher values of R will put more trust in the odometry
     * @param Q higher values of Q trusts the AprilTag more
     * @return this
     * @since 4.1.0
     */
    @NonNull
    public AprilTagRelocalizingAccumulator setKalmanGains(double R, double Q) {
        xf = new Filter.Kalman(R, Q);
        yf = new Filter.Kalman(R, Q);
        rf = new Filter.Kalman(R, Q);
        return this;
    }

    /**
     * Set whether to use the Kalman filter for the x, y, and r components.
     *
     * @param useKf whether to use the Kalman filter
     * @return this
     */
    @NonNull
    public AprilTagRelocalizingAccumulator setUseKalmanFilter(boolean useKf) {
        this.useKf = useKf;
        return this;
    }

    /**
     * Add a data filter that will apply to the detections of the processor whether to accept processing this tag.
     *
     * @param filter the filter to add
     * @return this
     * @since 5.1.0
     */
    @NonNull
    public AprilTagRelocalizingAccumulator addDataFilter(@NonNull Predicate<AprilTagData> filter) {
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
    @NonNull
    public AprilTagRelocalizingAccumulator removeDataFilter(@NonNull Predicate<AprilTagData> filter) {
        if (!filters.remove(filter))
            Dbg.warn(getClass(), "Unable to remove filter '%', not found.", filter);
        return this;
    }

    /**
     * Whether to set the drive pose to the vision estimate. Default is on.
     *
     * @param setPoseAutomatically whether AprilTagRelocalizingAccumulator is on
     * @return this
     */
    @NonNull
    public AprilTagRelocalizingAccumulator setActive(boolean setPoseAutomatically) {
        active = setPoseAutomatically;
        return this;
    }

    /**
     * Whether to set the drive pose to the vision estimate with heading information. Default is OFF.
     *
     * @param setHeadingAutomatically whether to also set the heading to the AprilTag estimate or not
     * @return this
     */
    @NonNull
    public AprilTagRelocalizingAccumulator setHeadingEstimate(boolean setHeadingAutomatically) {
        updateHeading = setHeadingAutomatically;
        return this;
    }

    /**
     * Run a standard accumulation while also relocalizing based on the presence of library AprilTags.
     *
     * @param twist the change in position and velocity with respect to time
     */
    public void accumulate(@NonNull Twist2dDual<Time> twist) {
        super.accumulate(twist);
        if (!active)
            return;

        ArrayList<AprilTagData> data = processor.getData();
        if (data.isEmpty()) {
            xf.setAccumulatedValue(pose.position.x);
            yf.setAccumulatedValue(pose.position.y);
            rf.setAccumulatedValue(pose.heading.toDouble());
            return;
        }

        estimates.clear();
        for (int i = 0; i < data.size(); i++) {
            AprilTagData aprilTag = data.get(i);
            Optional<Pose3D> robotPose = aprilTag.getRobotPose();

            if (robotPose.isEmpty() || filters.stream().anyMatch(f -> !f.test(aprilTag))) {
                // No luck with this ID
                continue;
            }

            // Future: latency compensation techniques

            // We don't care about the other rotational and translational properties of this pose so we ignore them
            Pose3D pose = robotPose.get();
            Position pos = pose.getPosition();
            // Rotate heading to match up with the field coordinate system
            estimates.add(new Pose2d(pos.x, pos.y, pose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI / 2));
        }

        // For multiple estimates, use the positionally closest pose to the robot's current pose for accuracy
        estimates.sort(Comparator.comparingDouble(q -> q.minus(pose).line.norm()));
        Pose2d newPose = estimates.get(0);

        Vector2d newVec = newPose.position;
        Rotation2d newHeading = newPose.heading;
        if (useKf) {
            Vector2d twistedTwist = pose.heading.times(twist.value().line);
            newVec = new Vector2d(
                    // Use deltas supplied directly from the pose twist to avoid integrating twice and causing oscillations
                    xf.calculateFromDelta(twistedTwist.x, newPose.position.x),
                    yf.calculateFromDelta(twistedTwist.y, newPose.position.y)
            );

            double currentHeading = newHeading.log();
            double headingTwist = twist.value().angle;
            if (Math.abs(currentHeading - lastHeading) > Math.PI) {
                // If we're at the heading modulus boundary, the filter should be reset and updated with the new heading
                headingTwist = pose.heading.inverse().log();
                rf.reset();
            }
            newHeading = Rotation2d.exp(rf.calculateFromDelta(headingTwist, currentHeading));
            lastHeading = currentHeading;
        }

        pose = new Pose2d(newVec, updateHeading ? newHeading : pose.heading);
    }
}
