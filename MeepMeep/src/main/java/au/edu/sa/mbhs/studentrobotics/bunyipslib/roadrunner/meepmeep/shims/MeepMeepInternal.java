package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Distance;
import kotlin.jvm.functions.Function2;

/**
 * Internal shim for MeepMeep and BunyipsLib interop.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
@SuppressWarnings("UnknownNullness")
public abstract class MeepMeepInternal {
    /**
     * Shim to reference a BunyipsLibBotBuilder instance that has been created and built.
     * This shim allows the usage of a TaskBuilder to create a trajectory action using units.
     * <p>
     * Do note this shim does not work if there are no BunyipsLibBotBuilders created and built,
     * and if multiple BunyipsLibBotBuilders are created and built, the <b>first one will be used</b> on a
     * bare makeTrajectory call. See the `on(int)` method to temporarily mention which BunyipsLibBotBuilder to use by
     * order of creation index.
     */
    public static final Drive drive = new Drive();
    /**
     * A reference to the current (dark mode) game field background for the current BunyipsLib release.
     */
    public static final MeepMeep.Background CURRENT_GAME_BACKGROUND_DARK = MeepMeep.Background.FIELD_DECODE_JUICE_DARK;

    /**
     * BunyipsLib MeepMeep drive shim.
     */
    public static class Drive {
        private final List<Function2<Pose2d, PoseMap, TrajectoryActionBuilder>> trajectoryActionBuilderSuppliers = new ArrayList<>();
        private final List<Supplier<Constraints>> constraintsSuppliers = new ArrayList<>();
        private final List<Supplier<DriveTrainType>> dttSuppliers = new ArrayList<>();
        private final List<Consumer<Action>> runActionConsumers = new ArrayList<>();
        private int operatingIndex;
        private boolean implicit;

        void __internalSetup(Function2<Pose2d, PoseMap, TrajectoryActionBuilder> trajectoryActionBuilderSupplier, Supplier<Constraints> constraintsSupplier, Supplier<DriveTrainType> dttSupplier, Consumer<Action> runActionConsumer) {
            trajectoryActionBuilderSuppliers.add(trajectoryActionBuilderSupplier);
            constraintsSuppliers.add(constraintsSupplier);
            dttSuppliers.add(dttSupplier);
            runActionConsumers.add(runActionConsumer);
        }

        /**
         * Tell the shim to use the BunyipsLibBotBuilder at the specified index in the order of creation.
         * This is temporary and will be reset back to the first shim after a trajectory is made.
         *
         * @param constructedDriveIndex the index of the BunyipsLibBotBuilder to use in the order they were built. Zero-indexed.
         * @return the Drive shim
         */
        public Drive on(int constructedDriveIndex) {
            if (constructedDriveIndex < 0 || constructedDriveIndex >= trajectoryActionBuilderSuppliers.size()) {
                throw new IllegalArgumentException("The constructedDriveIndex must be within the bounds of the BunyipsLibBotBuilders created!");
            }
            operatingIndex = constructedDriveIndex;
            return this;
        }

        /**
         * Declares for the next {@code makeTrajectory()} call to use an implicit pose constructor.
         * <p>
         * This method should be used to replicate the behaviour when using {@code makeTrajectory(poseMap)} in BunyipsLib,
         * where the inferred start pose will be doubly-mapped to preserve the absolute positioning of the pose (particularly for
         * Starting Configuration objects).
         * <p>
         * For example, a pattern like the following is when an implicit start pose is being used with a mapping:
         * <pre><code>
         * drive.setPose(START_POSE);
         * drive.makeTrajectory(POSE_MAP) // START_POSE will be mapped twice to preserve absolute positioning
         *    // ...
         *    .addTask();
         * </code></pre>
         * MeepMeep is incapable of performing this implicit conversion automatically without this method.
         * <p>
         * This method enables the "start pose" parsing for one call to {@code makeTrajectory()}. It is expected the pose map
         * you use is self-invertible as per the standard {@code makeTrajectory()} documentation. This method has no effect
         * if you don't use a pose map.
         *
         * @return the Drive shim, with start pose double-conversion enabled for the next builder (disabled after use)
         */
        public Drive useImplicitStartPose() {
            implicit = true;
            return this;
        }

        /**
         * Shim to create a new trajectory from the supplied pose.
         * <p>
         * Implicit poses are not supported in this shim, but if this start pose is the result of the absolute robot
         * position (e.g. {@code setPose}), consider calling {@link #useImplicitStartPose()} first to avoid mapping the start pose.
         *
         * @param startPose the pose to start the trajectory at and where the robot will be placed
         * @param poseMap   the pose map to use for the trajectory
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Pose2d startPose, PoseMap poseMap) {
            if (trajectoryActionBuilderSuppliers.isEmpty()) {
                throw new IllegalStateException("A BunyipsLibBotBuilder must be created and built before calling drive.makeTrajectory!");
            }
            if (implicit) {
                startPose = Geometry.map(startPose, poseMap);
                implicit = false;
            }
            MeepMeepTaskBuilder tb = new MeepMeepTaskBuilder(
                    trajectoryActionBuilderSuppliers.get(operatingIndex).invoke(startPose, poseMap),
                    constraintsSuppliers.get(operatingIndex).get(),
                    dttSuppliers.get(operatingIndex).get(),
                    runActionConsumers.get(operatingIndex)
            );
            operatingIndex = 0;
            return tb;
        }

        /**
         * Shim to create a new trajectory from the supplied pose.
         * <p>
         * Implicit poses are not supported in this shim, but if this start pose is the result of the absolute robot
         * position (e.g. {@code setPose}), consider calling {@link #useImplicitStartPose()} first to avoid mapping the start pose.
         *
         * @param startVec the vector to start the trajectory at and where the robot will be placed
         * @param distUnit the unit of distance of the start pose
         * @param ang      the angle of the start pose
         * @param angUnit  the unit of angle of the start pose
         * @param poseMap  the pose map to use for the trajectory
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Vector2d startVec, Distance distUnit, double ang, Angle angUnit, PoseMap poseMap) {
            return makeTrajectory(Geometry.poseFrom(startVec, distUnit, ang, angUnit), poseMap);
        }

        /**
         * Shim to create a new trajectory from the supplied pose. Implicitly uses the IdentityPoseMap.
         * <p>
         * Implicit poses are not supported in this shim, but if this start pose is the result of the absolute robot
         * position (e.g. {@code setPose}), consider calling {@link #useImplicitStartPose()} first to avoid mapping the start pose.
         *
         * @param startVec the vector to start the trajectory at and where the robot will be placed
         * @param distUnit the unit of distance of the start pose
         * @param ang      the angle of the start pose
         * @param angUnit  the unit of angle of the start pose
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Vector2d startVec, Distance distUnit, double ang, Angle angUnit) {
            return makeTrajectory(Geometry.poseFrom(startVec, distUnit, ang, angUnit), new IdentityPoseMap());
        }

        /**
         * Shim to create a new trajectory from the supplied pose. Implicitly uses the IdentityPoseMap.
         * <p>
         * Implicit poses are not supported in this shim, but if this start pose is the result of the current robot
         * position (e.g. {@code setPose}), consider calling {@link #useImplicitStartPose()} first to avoid mapping the start pose.
         *
         * @param startPose the pose to start the trajectory at and where the robot will be placed
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Pose2d startPose) {
            return makeTrajectory(startPose, new IdentityPoseMap());
        }
    }
}
