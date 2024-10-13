package org.murraybridgebunyips.bunyipslib.roadrunner.messages;

import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

/**
 * RoadRunner v1.0 logging message for a drive command.
 *
 * @since 6.0.0
 */
public final class DriveCommandMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The forward velocity at this time.
     */
    public double forwardVelocity;
    /**
     * The forward acceleration at this time.
     */
    public double forwardAcceleration;
    /**
     * The lateral velocity at this time.
     */
    public double lateralVelocity;
    /**
     * The lateral acceleration at this time.
     */
    public double lateralAcceleration;
    /**
     * The angular velocity at this time.
     */
    public double angularVelocity;
    /**
     * The angular acceleration at this time.
     */
    public double angularAcceleration;

    @SuppressWarnings("MissingJavadoc")
    public DriveCommandMessage(PoseVelocity2dDual<Time> poseVelocity) {
        timestamp = System.nanoTime();
        forwardVelocity = poseVelocity.linearVel.x.get(0);
        forwardAcceleration = poseVelocity.linearVel.x.get(1);
        lateralVelocity = poseVelocity.linearVel.y.get(0);
        lateralAcceleration = poseVelocity.linearVel.y.get(1);
        angularVelocity = poseVelocity.angVel.get(0);
        angularAcceleration = poseVelocity.angVel.get(1);
    }
}
