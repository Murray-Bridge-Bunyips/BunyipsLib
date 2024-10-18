package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;

/**
 * Interface implemented by all RoadRunner drive classes.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public interface RoadRunnerDrive extends Moveable {
    /**
     * The constants object represents all configuration options required for creating
     * a RoadRunner trajectory. These constants are used internally by [.makeTrajectory] to provide
     * task creation.
     *
     * @return the constants and method references used to construct trajectories and motion profiles
     */
    Constants getConstants();

    /**
     * Dispatch loop for localization and drive power updates.
     */
    void periodic();

    /**
     * Begin building a RoadRunner trajectory from the last-known robot position when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory() {
        return new TaskBuilder(getConstants(), Storage.memory().lastKnownPosition, new IdentityPoseMap());
    }

    /**
     * Begin building a RoadRunner trajectory from the last-known robot position when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @param poseMap the PoseMap to use for this builder
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory(PoseMap poseMap) {
        return new TaskBuilder(getConstants(), Storage.memory().lastKnownPosition, poseMap);
    }

    /**
     * Begin building a RoadRunner trajectory from the supplied pose when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @param startPose the pose to start the trajectory generation from
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory(Pose2d startPose) {
        return new TaskBuilder(getConstants(), startPose, new IdentityPoseMap());
    }

    /**
     * Begin building a RoadRunner trajectory from the supplied pose when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @param startPose the pose to start the trajectory generation from
     * @param poseMap   the PoseMap to use for this builder
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory(Pose2d startPose, PoseMap poseMap) {
        return new TaskBuilder(getConstants(), startPose, poseMap);
    }
}