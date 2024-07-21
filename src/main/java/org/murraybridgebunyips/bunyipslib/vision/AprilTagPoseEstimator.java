package org.murraybridgebunyips.bunyipslib.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.ArrayList;

/**
 * Combines an AprilTag processor and RoadRunner drive to supply updates in pose estimation.
 *
 * @author Lucas Bubner, 2024
 */
public class AprilTagPoseEstimator {
    private final AprilTag processor;
    private final RoadRunnerDrive drive;
    private boolean active = true;

    /**
     * Constructor for AprilTagPoseEstimator.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param drive     RoadRunner drive
     */
    public AprilTagPoseEstimator(AprilTag processor, RoadRunnerDrive drive) {
        this.processor = processor;
        this.drive = drive;
    }

    /**
     * Whether to set the drive pose to the vision estimate. Default is on.
     *
     * @param setPoseAutomatically whether AprilTagPoseEstimator is on
     */
    public void setActive(boolean setPoseAutomatically) {
        active = setPoseAutomatically;
    }

    /**
     * Propagate interpretation of AprilTag processor and set the pose estimate to the tag, if
     * available by the SDK. This method will no-op if insufficient information is available.
     */
    public void update() {
        if (!active || !processor.isRunning())
            return;

        ArrayList<AprilTagData> data = processor.getData();
        if (data.isEmpty())
            return;

        // We will simply rely on the first entry in the list of detected tags
        AprilTagData aprilTag = data.get(0);
        VectorF tagPos = aprilTag.getFieldPosition();

        if (tagPos == null || !aprilTag.isFtcPoseAvailable())
            return;
        assert aprilTag.getX() != null && aprilTag.getY() != null;

        // Calculate displacement vector in field coordinates
        // Will rotate 90 degrees by swapping and negating x and y as the drive has x forward and y left (unit circle)
        Vector2d pos = new Vector2d(
                -aprilTag.getY() + tagPos.get(0),
                -aprilTag.getX() + tagPos.get(1)
        );

        // Will not set heading, as the IMU is more reliable due to how AprilTags are interpreted.
        // Most times, we will know what way we are facing with certainty, where encoders drift but IMU
        // drift is less of a concern especially in the span of a single match
        // Future: Can integrate heading with a Kalman filter or similar to allow it to be used (as heading is the most
        //   unreliable part of the pose estimate)
        Pose2d newPose = new Pose2d(pos.getX(), pos.getY(), drive.getPoseEstimate().getHeading());
        // Avoid spamming the logs by logging the events that are over an inch away from the current estimate
        if (drive.getPoseEstimate().vec().distTo(newPose.vec()) >= 1)
            Dbg.logd(getClass(), "Updated pose based on AprilTag ID#%, %->%", aprilTag.getId(), drive.getPoseEstimate(), newPose);
        drive.setPoseEstimate(newPose);
    }
}
