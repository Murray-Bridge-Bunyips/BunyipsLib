package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * RoadRunner v1.0 logging message for a current pose or pose velocity.
 *
 * @since 6.0.0
 */
public final class PoseMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The x position at this time.
     */
    public double x;
    /**
     * The y position at this time.
     */
    public double y;
    /**
     * The heading at this time.
     */
    public double heading;

    @SuppressWarnings("MissingJavadoc")
    public PoseMessage(@NonNull Pose2d pose) {
        timestamp = System.nanoTime();
        x = pose.position.x;
        y = pose.position.y;
        heading = pose.heading.toDouble();
    }

    @SuppressWarnings("MissingJavadoc")
    public PoseMessage(@NonNull PoseVelocity2d poseVelocity) {
        timestamp = System.nanoTime();
        x = poseVelocity.linearVel.x;
        y = poseVelocity.linearVel.y;
        heading = poseVelocity.angVel; // we don't normalise this way
    }
}

