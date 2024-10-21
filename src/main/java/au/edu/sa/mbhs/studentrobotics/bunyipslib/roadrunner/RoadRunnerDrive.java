package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
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
     * <p>
     * Note: If this drive is a subsystem, this method is called in the subsystem's {@code update()} method and
     * should be called there.
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

    /**
     * Begin building a RoadRunner trajectory from the supplied pose when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @param startVec the vector to start the trajectory at and where the robot will be placed
     * @param distUnit the unit of distance of the start pose
     * @param ang      the angle of the start pose
     * @param angUnit  the unit of angle of the start pose
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory(Vector2d startVec, Distance distUnit, double ang, Angle angUnit) {
        return new TaskBuilder(getConstants(), Geometry.unitPose(startVec, distUnit, ang, angUnit), new IdentityPoseMap());
    }

    /**
     * Begin building a RoadRunner trajectory from the supplied pose when this method is called.
     * For deferring this starting pose dynamically, consider a DynamicTask (util. `Task.defer`) builder.
     *
     * @param startVec the vector to start the trajectory at and where the robot will be placed
     * @param distUnit the unit of distance of the start pose
     * @param ang      the angle of the start pose
     * @param angUnit  the unit of angle of the start pose
     * @param poseMap   the PoseMap to use for this builder
     * @return extended RoadRunner trajectory task builder
     */
    default TaskBuilder makeTrajectory(Vector2d startVec, Distance distUnit, double ang, Angle angUnit, PoseMap poseMap) {
        return new TaskBuilder(getConstants(), Geometry.unitPose(startVec, distUnit, ang, angUnit), poseMap);
    }
}