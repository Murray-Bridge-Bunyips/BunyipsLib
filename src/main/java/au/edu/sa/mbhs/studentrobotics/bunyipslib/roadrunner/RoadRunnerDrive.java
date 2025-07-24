package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.accumulators.Accumulator;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters.Constants;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Storage;
import dev.frozenmilk.util.cell.RefCell;

/**
 * Interface implemented by all RoadRunner drive classes.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public interface RoadRunnerDrive extends Moveable {
    /**
     * The constants object represents all configuration options required for creating
     * a RoadRunner trajectory. These constants are used internally by {@link #makeTrajectory} to provide
     * task creation.
     *
     * @return the constants and method references used to construct trajectories and motion profiles
     */
    @NonNull
    Constants getConstants();

    /**
     * Dispatch loop for localization and drive power updates.
     * <p>
     * Note: If this drive is a subsystem, this method is called in the subsystem's {@code update()} method and
     * should be called there.
     */
    void periodic();

    /**
     * Set the localizer for this drive.
     *
     * @param localizer the localizer to use
     * @return this drive
     */
    @NonNull
    RoadRunnerDrive withLocalizer(@NonNull Localizer localizer);

    /**
     * Get the localizer for this drive.
     *
     * @return the localizer
     */
    @NonNull
    Localizer getLocalizer();

    /**
     * Set the localizer for this drive.
     *
     * @param localizer the localizer to use
     * @return this drive
     */
    @NonNull
    default RoadRunnerDrive setLocalizer(@NonNull Localizer localizer) {
        return withLocalizer(localizer);
    }

    /**
     * Set the accumulator for this drive.
     *
     * @param accumulator the accumulator to use that will accumulate the localizer twist
     * @return this drive
     */
    @NonNull
    RoadRunnerDrive withAccumulator(@NonNull Accumulator accumulator);

    /**
     * Get the accumulator for this drive.
     *
     * @return the accumulator
     */
    @NonNull
    Accumulator getAccumulator();

    /**
     * Set the accumulator for this drive.
     *
     * @param accumulator the accumulator to use that will accumulate the localizer twist
     * @return this drive
     */
    @NonNull
    default RoadRunnerDrive setAccumulator(@NonNull Accumulator accumulator) {
        return withAccumulator(accumulator);
    }

    /**
     * Builds a RoadRunner trajectory starting from the robot's last-known position <b>at the time of this method being called.</b>
     * Users should be aware that improperly calling this method multiple times can create trajectories that start from the wrong spot,
     * as the start pose used will be the one set during initialisation and <b>*NOT*</b> the endpoint of the last trajectory.
     * <p>
     * The start pose of this trajectory is captured implicitly from {@link Storage#memory()}. For chaining or splicing trajectories,
     * see {@link TaskBuilder#fresh()} or using a {@link RefCell} as a parameter to {@code addTask} or {@code build} to keep track
     * of the end position of the last trajectory. For runtime pose evaluation, use {@link Task#defer}.
     * <p>
     * See the RoadRunner section in the wiki for more details, and ensure to review your trajectories on a Field dashboard.
     *
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory() {
        return new TaskBuilder(getConstants(), Storage.memory().lastKnownPosition, new IdentityPoseMap());
    }

    /**
     * Builds a RoadRunner trajectory starting from the robot's last-known position <b>at the time of this method being called.</b>
     * Users should be aware that improperly calling this method multiple times can create trajectories that start from the wrong spot,
     * as the start pose used will be the one set during initialisation and <b>*NOT*</b> the endpoint of the last trajectory.
     * <p>
     * The start pose of this trajectory is captured implicitly from {@link Storage#memory()}. For chaining or splicing trajectories,
     * see {@link TaskBuilder#fresh()} or using a {@link RefCell} as a parameter to {@code addTask} or {@code build} to keep track
     * of the end position of the last trajectory. For runtime pose evaluation, use {@link Task#defer}.
     * <p>
     * See the RoadRunner section in the wiki for more details, and ensure to review your trajectories on a Field dashboard.
     *
     * @param poseMap the PoseMap to use for this builder, note that the implicit last-known position is used and
     *                automatically passed into the PoseMap now to later 'invert' it. <b>Using this method assumes your PoseMap is self-invertible.</b>
     *                For most mappings, this will result in the absolute last-known position being used as the starting pose, which
     *                is usually the case when you are trying to use the current position of the robot.
     *                If you wish the PoseMap to apply normally, consider using the other makeTrajectory methods
     *                and manually passing the last-known position from {@link Storage}.
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory(@NonNull PoseMap poseMap) {
        return new TaskBuilder(getConstants(), poseMap.map(Pose2dDual.constant(Storage.memory().lastKnownPosition, 1)).value(), poseMap);
    }

    /**
     * Builds a RoadRunner trajectory from an explicit start pose. This method yields the best accuracy.
     * <p>
     * Use this for precise control over the trajectory's starting position.
     * <p>
     * For chaining or splicing trajectories, see {@link TaskBuilder#fresh()} or using a {@link RefCell} to splice
     * when {@code addTask} or {@code build} is called. For runtime pose evaluation, use {@link Task#defer}.
     * See the RoadRunner section in the wiki for more details.
     *
     * @param startPose the explicit start {@link Pose2d} for this trajectory
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory(@NonNull Pose2d startPose) {
        return new TaskBuilder(getConstants(), startPose, new IdentityPoseMap());
    }

    /**
     * Builds a RoadRunner trajectory from an explicit start pose, mapped through a {@link PoseMap}.
     * <p>
     * For chaining or splicing trajectories, see {@link TaskBuilder#fresh()} or using a {@link RefCell} to splice
     * when {@code addTask} or {@code build} is called. For runtime pose evaluation, use {@link Task#defer}.
     * See the RoadRunner section in the wiki for more details.
     *
     * @param startPose the explicit start {@link Pose2d} for this trajectory
     * @param poseMap   the {@link PoseMap} to transform the start pose
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory(@NonNull Pose2d startPose, @NonNull PoseMap poseMap) {
        return new TaskBuilder(getConstants(), startPose, poseMap);
    }

    /**
     * Builds a RoadRunner trajectory from a vector and angle, with explicit units.
     * <p>
     * For chaining or splicing trajectories, see {@link TaskBuilder#fresh()} or using a {@link RefCell} to splice
     * when {@code addTask} or {@code build} is called. For runtime pose evaluation, use {@link Task#defer}.
     * See the RoadRunner section in the wiki for more details.
     *
     * @param startVec the start {@link Vector2d} of this trajectory
     * @param distUnit the {@link Distance} unit
     * @param ang      the angle value
     * @param angUnit  the {@link Angle} unit
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory(@NonNull Vector2d startVec, @NonNull Distance distUnit, double ang, @NonNull Angle angUnit) {
        return new TaskBuilder(getConstants(), Geometry.poseFrom(startVec, distUnit, ang, angUnit), new IdentityPoseMap());
    }

    /**
     * Builds a RoadRunner trajectory from a vector and angle, with explicit units and pose mapping.
     * <p>
     * For chaining or splicing trajectories, see {@link TaskBuilder#fresh()} or using a {@link RefCell} to splice
     * when {@code addTask} or {@code build} is called. For runtime pose evaluation, use {@link Task#defer}.
     * See the RoadRunner section in the wiki for more details.
     *
     * @param startVec the start {@link Vector2d} of this trajectory
     * @param distUnit the {@link Distance} unit
     * @param ang      the angle value
     * @param angUnit  the {@link Angle} unit
     * @param poseMap  the {@link PoseMap} to transform the start pose
     * @return a {@link TaskBuilder} for trajectory creation
     */
    @NonNull
    default TaskBuilder makeTrajectory(@NonNull Vector2d startVec, @NonNull Distance distUnit, double ang, @NonNull Angle angUnit, @NonNull PoseMap poseMap) {
        return new TaskBuilder(getConstants(), Geometry.poseFrom(startVec, distUnit, ang, angUnit), poseMap);
    }
}