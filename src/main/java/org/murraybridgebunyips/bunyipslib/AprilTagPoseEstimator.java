package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.ArrayList;

/**
 * Combines an AprilTag processor and RoadRunner drive to supply updates in pose estimation.
 * Work In Progress
 *
 * @author Lucas Bubner, 2024
 */
public class AprilTagPoseEstimator {
    private final AprilTag processor;
    private final RoadRunnerDrive drive;

    /**
     * Constructor for AprilTagPoseEstimator.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param drive RoadRunner drive
     */
    public AprilTagPoseEstimator(AprilTag processor, RoadRunnerDrive drive) {
        this.processor = processor;
        if (!this.processor.isRunning())
            throw new EmergencyStop("AprilTag processor is not attached to a Vision instance");
        this.drive = drive;
    }

    /**
     * Propagate interpretation of AprilTag processor and set the pose estimate to the tag, if
     * available by the SDK. This method will no-op if insufficient information is available.
     */
    public void update() {
        ArrayList<AprilTagData> data = processor.getData();
        if (data.isEmpty())
            return;

        // TODO: experimental
        // For now we will rely on simply the first entry
        AprilTagData entry = data.get(0);
        VectorF tagPos = entry.getFieldPosition();
//        Quaternion camOri = entry.getFieldOrientation();

        if (tagPos == null || !entry.isFtcPoseAvailable())
            return;
        assert entry.getX() != null && entry.getY() != null;

        Vector2d pos = new Vector2d(
                entry.getY() - tagPos.get(0),
                entry.getX() - tagPos.get(1)
        );

        drive.setPoseEstimate(new Pose2d(-pos.getX(), -pos.getY(), drive.getPoseEstimate().getHeading()));
    }
}
