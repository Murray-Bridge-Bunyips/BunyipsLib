package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
     * BunyipsLib MeepMeep drive shim.
     */
    public static class Drive {
        private final List<Function2<Pose2d, PoseMap, TrajectoryActionBuilder>> trajectoryActionBuilderSuppliers = new ArrayList<>();
        private final List<Supplier<Constraints>> constraintsSuppliers = new ArrayList<>();
        private final List<Supplier<DriveTrainType>> dttSuppliers = new ArrayList<>();
        private final List<Consumer<Action>> runActionConsumers = new ArrayList<>();
        private int operatingIndex;

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
         * Shim to create a new trajectory from the supplied pose.
         * Implicit poses are not supported in this shim.
         *
         * @param startPose the pose to start the trajectory at and where the robot will be placed
         * @param poseMap   the pose map to use for the trajectory
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Pose2d startPose, PoseMap poseMap) {
            if (trajectoryActionBuilderSuppliers.isEmpty()) {
                throw new IllegalStateException("A BunyipsLibBotBuilder must be created and built before calling drive.makeTrajectory!");
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
         * Implicit poses are not supported in this shim.
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
         * Implicit poses are not supported in this shim.
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
         * Implicit poses are not supported in this shim.
         *
         * @param startPose the pose to start the trajectory at and where the robot will be placed
         * @return a new TaskBuilder to build the trajectory action
         */
        public MeepMeepTaskBuilder makeTrajectory(Pose2d startPose) {
            return makeTrajectory(startPose, new IdentityPoseMap());
        }
    }
}
