package au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;

import java.util.LinkedList;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages.PoseMessage;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * Accumulates pose estimates from a {@link Localizer} to expose {@link Localizable} properties.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@Config
public class Accumulator implements Localizable {
    /**
     * The maximum pose history length that should be stored in an accumulator.
     */
    public static int MAX_POSE_HISTORY = 100;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private LinkedList<Pose2d> poseHistory = new LinkedList<>();

    protected Pose2d pose;
    protected PoseVelocity2d velocity = Geometry.zeroVel();

    /**
     * Construct an Accumulator base.
     *
     * @param initialPose the initial pose to accumulate from
     */
    public Accumulator(Pose2d initialPose) {
        pose = initialPose;
    }

    /**
     * Create an Accumulator base with the last known robot position.
     * Useful for constructing an accumulator that will be overridden by replacing an Accumulator.
     */
    public Accumulator() {
        pose = Storage.memory().lastKnownPosition;
    }

    /**
     * Run one accumulation of this accumulator to update the pose with this delta.
     * The pose and pose history on FtcDashboard will also be updated on this accumulation.
     *
     * @param twist the change in position and velocity with respect to time
     */
    public void accumulate(Twist2dDual<Time> twist) {
        pose = pose.plus(twist.value());
        velocity = twist.velocity().value();
        Storage.memory().lastKnownPosition = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > MAX_POSE_HISTORY) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(pose));

        Dashboard.usePacket(p -> {
            p.put("x (in)", pose.position.x);
            p.put("y (in)", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            p.put("xVel (in/s)", velocity.linearVel.x);
            p.put("yVel (in/s)", velocity.linearVel.y);
            p.put("headingVel (deg/s)", Math.toDegrees(velocity.angVel));

            Canvas c = p.fieldOverlay();
            c.setStrokeWidth(1);
            c.setStroke("#3F51B5");
            Dashboard.drawPoseHistory(c, poseHistory);
            Dashboard.drawRobot(c, pose);

            Vector2d velocityDirection = pose.heading
                    .times(velocity)
                    .linearVel;
            c.setStroke("#751000")
                    .strokeLine(
                            pose.position.x,
                            pose.position.y,
                            pose.position.x + velocityDirection.x,
                            pose.position.y + velocityDirection.y
                    );
        });
    }

    /**
     * Copy the state of this accumulator to another accumulator.
     *
     * @param other the other accumulator to copy to
     */
    public final void copyTo(Accumulator other) {
        other.pose = pose;
        other.velocity = velocity;
        other.poseHistory = poseHistory;
    }

    /**
     * @return a clone of the pose history as accumulated by this accumulator
     */
    @SuppressWarnings("unchecked")
    public final LinkedList<Pose2d> getPoseHistory() {
        return (LinkedList<Pose2d>) poseHistory.clone();
    }

    @NonNull
    @Override
    public final Pose2d getPoseEstimate() {
        return pose;
    }

    @Override
    public final void setPoseEstimate(@NonNull Pose2d newPose) {
        pose = newPose;
    }

    @NonNull
    @Override
    public final PoseVelocity2d getPoseVelocity() {
        return velocity;
    }
}
