package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.Predicate;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Filter;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
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
    public static double DEFAULT_Q = 1.0e-4;

    private final AprilTag processor;
    private final HashSet<Predicate<AprilTagData>> filters = new HashSet<>();
    private final ArrayList<Pose2d> estimates = new ArrayList<>();

    private Filter.Kalman xf;
    private Filter.Kalman yf;
    private Filter.Kalman rf;

    private boolean active = true;
    private boolean updateHeading = true;

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
    public AprilTagRelocalizingAccumulator setKalmanGains(double R, double Q) {
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
    public AprilTagRelocalizingAccumulator setActive(boolean setPoseAutomatically) {
        active = setPoseAutomatically;
        return this;
    }

    /**
     * Whether to set the drive pose to the vision estimate with heading information. Default is ON.
     *
     * @param setHeadingAutomatically whether to also set the heading to the AprilTag estimate or not
     * @return this
     */
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
        if (data.isEmpty())
            return;

        estimates.clear();
        for (int i = 0; i < data.size(); i++) {
            AprilTagData aprilTag = data.get(i);
            Optional<Pose3D> robotPose = aprilTag.getRobotPose();

            if (!robotPose.isPresent() || filters.stream().anyMatch(f -> !f.test(aprilTag))) {
                // No luck with this ID
                continue;
            }

            // Future: latency compensation techniques

            // Need to convert this pose from Cartesian to Robot coordinate frames - we also don't care about
            // the other rotational and translational properties of this pose so we ignore them
            Pose3D pose = robotPose.get();
            // noinspection SuspiciousNameCombination
            double x = pose.getPosition().y;
            double y = -pose.getPosition().x;
            double r = pose.getOrientation().getYaw(AngleUnit.RADIANS);

            // We also need to rotate the entire pose by 90 degrees to match the coordinate systems up
            estimates.add(new Pose2d(Rotation2d.exp(Math.PI / 2).times(new Vector2d(x, y)), r + Math.PI / 2));
        }

        // Take averages based on all the estimates we collected, then apply it to the current pose
        Vector2d positionAvg = estimates.stream()
                .map(p -> p.position)
                .reduce(Geometry.zeroVec(), Vector2d::plus)
                .div(estimates.size());
        double headingAvgRad = estimates.stream()
                .map(p -> p.heading.toDouble())
                .reduce(0.0, Double::sum) / estimates.size();

        Vector2d twistedTwist = pose.heading.times(twist.value().line);
        Vector2d kfVec = new Vector2d(
                // Use deltas supplied directly from the pose twist to avoid integrating twice and causing oscillations
                xf.calculateFromDelta(twistedTwist.x, positionAvg.x),
                yf.calculateFromDelta(twistedTwist.y, positionAvg.y)
        );
        // TODO: headingAvgRad has wrapping problems at pi and -pi
        Rotation2d kfHeading = Rotation2d.exp(rf.calculateFromDelta(twist.value().angle, headingAvgRad));

        pose = new Pose2d(kfVec, updateHeading ? kfHeading : pose.heading);
    }
}
