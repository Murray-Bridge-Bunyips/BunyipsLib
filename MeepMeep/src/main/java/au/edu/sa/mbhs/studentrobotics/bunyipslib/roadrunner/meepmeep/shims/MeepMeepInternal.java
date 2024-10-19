package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.TaskBuilder;

/**
 * Internal shim for MeepMeep and BunyipsLib interop.
 *
 * @author Lucas Bubner, 2024
 */
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
    public static Drive drive = new Drive();

    /**
     * BunyipsLib MeepMeep drive shim.
     */
    public static class Drive {
        private final List<Function<Pose2d, TrajectoryActionBuilder>> trajectoryActionBuilderSuppliers = new ArrayList<>();
        private final List<Supplier<Constraints>> constraintsSuppliers = new ArrayList<>();
        private final List<Supplier<DriveTrainType>> dttSuppliers = new ArrayList<>();
        private final List<Consumer<Action>> runActionConsumers = new ArrayList<>();
        private int operatingIndex;

        void __internalSetup(Function<Pose2d, TrajectoryActionBuilder> trajectoryActionBuilderSupplier, Supplier<Constraints> constraintsSupplier, Supplier<DriveTrainType> dttSupplier, Consumer<Action> runActionConsumer) {
            trajectoryActionBuilderSuppliers.add(trajectoryActionBuilderSupplier);
            constraintsSuppliers.add(constraintsSupplier);
            dttSuppliers.add(dttSupplier);
            runActionConsumers.add(runActionConsumer);
        }

        /**
         * Tell the shim to use the BunyipsLibBotBuilder at the specified index in the order of creation.
         * This is temporary and will be reset back to the first shim after a trajectory is made.
         *
         * @param constructedDriveIndex the index of the BunyipsLibBotBuilder to use in the order they were built
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
         *
         * @param startPose the pose to start the trajectory at and where the robot will be placed
         * @return a new TaskBuilder to build the trajectory action
         */
        public TaskBuilder makeTrajectory(Pose2d startPose) {
            if (trajectoryActionBuilderSuppliers.isEmpty()) {
                throw new IllegalStateException("A BunyipsLibBotBuilder must be created and built before calling drive.makeTrajectory!");
            }
            TaskBuilder tb = new TaskBuilder(
                    trajectoryActionBuilderSuppliers.get(operatingIndex).apply(startPose),
                    constraintsSuppliers.get(operatingIndex).get(),
                    dttSuppliers.get(operatingIndex).get(),
                    runActionConsumers.get(operatingIndex)
            );
            operatingIndex = 0;
            return tb;
        }
    }
}