package org.murraybridgebunyips.bunyipslib.roadrunner.messages;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * RoadRunner v1.0 logging message for a current pose.
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
    public PoseMessage(Pose2d pose) {
        timestamp = System.nanoTime();
        x = pose.position.x;
        y = pose.position.y;
        heading = pose.heading.toDouble();
    }
}

