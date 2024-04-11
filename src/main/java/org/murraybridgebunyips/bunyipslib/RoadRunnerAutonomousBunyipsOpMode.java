package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;

import java.util.ArrayList;

/**
 * RoadRunnerAutonomousBunyipsOpMode (RRABOM, nickname "Rabone").
 * Superset of {@link AutonomousBunyipsOpMode} that integrates RoadRunner trajectories into the task queue through
 * utility methods such as {@code addNewTrajectory}.
 *
 * @param <T> RoadRunner drive instance
 * @author Lucas Bubner, 2023
 * @see BunyipsOpMode
 */
@Config
public abstract class RoadRunnerAutonomousBunyipsOpMode<T extends RoadRunnerDrive> extends AutonomousBunyipsOpMode {
    /**
     * Default timeout for all RoadRunner tasks, if not explicitly mentioned.
     */
    public static Measure<Time> DEFAULT_TIMEOUT = INFINITE_TIMEOUT;
    private final ArrayList<RoadRunnerTask<T>> rrTasks = new ArrayList<>();

    /**
     * Drive instance to be used for RoadRunner trajectories.
     * This is automatically set by the {@link #setDrive()} method, ensuring that the drive instance
     * is set before any tasks are added to the queue.
     * <p>
     * {@code drive = new MecanumDrive(...)}
     */
    protected T drive;

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised. You may also add specific tasks to the queue
     * here, but it is recommended to use {@link #setInitTask(RobotTask)} or {@link #onReady(OpModeSelection)} instead.
     */
    protected abstract void onInitialise();

    /**
     * Set the drive instance to be used for RoadRunner trajectories. This method ensures
     * that the drive instance is set to avoid accidental NullPointerExceptions, similar to how
     * setOpModes and setInitTask are handled. This is called after initialisation, so your config
     * instance will be available.
     *
     * @return RoadRunner drive instance, can be instantiated here or as a class member
     */
    protected abstract T setDrive();

    @Override
    protected final void onInitialisation() {
        onInitialise();
        assertDrive();
    }

    private void assertDrive() {
        if (drive != null) return;
        drive = setDrive();
        if (drive == null)
            throw new NullPointerException("drive instance is not set!");
    }

    private Pose2d getPreviousPose() {
        // Needed to splice the last pose from the last trajectory
        return rrTasks.isEmpty() ? drive.getPoseEstimate() : rrTasks.get(rrTasks.size() - 1).getEndPose();
    }

    /**
     * STRONGLY RECOMMENDED: Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control ({@link RoadRunnerTrajectoryTaskBuilder#withTimeout(Measure)}).
     * <p>
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, rad)
     * @return Builder for the trajectory
     */
    protected final RoadRunnerTrajectoryTaskBuilder addNewTrajectory(Pose2d startPose) {
        assertDrive();
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        drive.setPoseEstimate(startPose);
        return new RoadRunnerTrajectoryTaskBuilder(startPose, builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    /**
     * STRONGLY RECOMMENDED: Use this method to build a new RoadRunner trajectory to the queue.
     * Creates a new builder for a RoadRunner trajectory, which will automatically add a
     * task to the queue when build() is called, optionally with a timeout control.
     * This method is the combination of {@link #newTrajectorySequence()} and {@link #addTrajectory(TrajectorySequence)},
     * using a custom builder that supports {@code setTimeout()} and priority building.
     * Without arguments, will use the current pose estimate of the drive or the last spliced pose.
     *
     * @return Builder for the trajectory
     * @see #addNewTrajectory(Pose2d)
     */
    protected final RoadRunnerTrajectoryTaskBuilder addNewTrajectory() {
        assertDrive();
        // noinspection rawtypes
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(getPreviousPose());
        return new RoadRunnerTrajectoryTaskBuilder(getPreviousPose(), builder.getBaseVelConstraint(), builder.getBaseAccelConstraint(), builder.getBaseTurnConstraintMaxAngVel(), builder.getBaseTurnConstraintMaxAngAccel());
    }

    /**
     * Create a new builder for a RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @return Builder for the trajectory
     * @see #newTrajectory(Pose2d)
     */
    protected final TrajectoryBuilder newTrajectory() {
        assertDrive();
        return drive.trajectoryBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(Trajectory)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, rad)
     * @return Builder for the trajectory
     */
    protected final TrajectoryBuilder newTrajectory(Pose2d startPose) {
        assertDrive();
        drive.setPoseEstimate(startPose);
        return drive.trajectoryBuilder(startPose);
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @return Builder for the trajectory
     * @see #newTrajectorySequence(Pose2d)
     */
    @SuppressWarnings("rawtypes")
    protected final TrajectorySequenceBuilder newTrajectorySequence() {
        assertDrive();
        return drive.trajectorySequenceBuilder(getPreviousPose());
    }

    /**
     * Create a new builder for a standard RoadRunner trajectory sequence using the drive system.
     * This will use the builder directly from RoadRunner, see {@link #addNewTrajectory(Pose2d)} for
     * integrated task building, or if you prefer you can use {@link #addTrajectory(TrajectorySequence)}
     * to add your sequence to the queue manually.
     *
     * @param startPose Starting pose of the trajectory, ** WILL SET DRIVE POSE ESTIMATE TO THIS POSE **, (in, in, rad)
     * @return Builder for the trajectory
     */
    @SuppressWarnings("rawtypes")
    protected final TrajectorySequenceBuilder newTrajectorySequence(Pose2d startPose) {
        assertDrive();
        drive.setPoseEstimate(startPose);
        return drive.trajectorySequenceBuilder(startPose);
    }

    /**
     * Add a trajectory to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory to add
     * @return Builder for the task
     */
    protected final AddTrajectoryBuilder<Trajectory> addTrajectory(Trajectory trajectory) {
        return new AddTrajectoryBuilder<>(trajectory);
    }

    /**
     * Add a trajectory sequence to the task queue with a timeout and priority level.
     *
     * @param trajectory Trajectory sequence to add
     * @return Builder for the task
     */
    protected final AddTrajectoryBuilder<TrajectorySequence> addTrajectory(TrajectorySequence trajectory) {
        return new AddTrajectoryBuilder<>(trajectory);
    }

    /**
     * Priority representation for building tasks.
     * LAST: Add the task to the end of the queue after the onReady() init callback has fired
     * NORMAL: Add the task to the queue immediately (default)
     * FIRST: Add the task to the front of the queue after the onReady() init callback has fired
     */
    protected enum PriorityLevel {
        LAST,
        NORMAL,
        FIRST//® Tech Challenge
    }

    protected final class AddTrajectoryBuilder<S> {
        private final S trajectory;
        private Measure<Time> timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
        private String name = null;

        public AddTrajectoryBuilder(S trajectory) {
            if (trajectory == null)
                throw new NullPointerException("trajectory cannot be null!");
            if (!(trajectory instanceof Trajectory) && !(trajectory instanceof TrajectorySequence))
                throw new EmergencyStop("trajectory must be a Trajectory or TrajectorySequence!");
            this.trajectory = trajectory;
        }

        /**
         * Set a timeout for the trajectory, to be applied to the overhead task running the trajectory.
         *
         * @param interval Timeout for the trajectory
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withTimeout(Measure<Time> interval) {
            timeout = interval;
            return this;
        }

        /**
         * Set the priority level of the task.
         *
         * @param p Priority level
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withPriority(PriorityLevel p) {
            priority = p;
            return this;
        }

        /**
         * Set the task name of the trajectory to show up in the telemetry.
         *
         * @param taskName Name of the task
         * @return trajectory builder
         */
        public AddTrajectoryBuilder<S> withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Add the trajectory to the task queue based on the builder arguments.
         */
        public void build() {
            assertDrive();
            RoadRunnerTask<T> task = null;
            if (trajectory instanceof Trajectory) {
                task = new RoadRunnerTask<>(timeout, drive, (Trajectory) trajectory);
            } else if (trajectory instanceof TrajectorySequence) {
                task = new RoadRunnerTask<>(timeout, drive, (TrajectorySequence) trajectory);
            }
            assert task != null;
            task.withName(name);
            rrTasks.add(task);
            switch (priority) {
                case LAST:
                    addTaskLast(task);
                    break;
                case NORMAL:
                    addTask(task);
                    break;
                case FIRST:
                    addTaskFirst(task);
                    break;
            }
        }
    }

    /**
     * Builder class for a RoadRunner trajectory, which supports adding the trajectory to the Task queue.
     */
    protected final class RoadRunnerTrajectoryTaskBuilder extends TrajectorySequenceBuilder<RoadRunnerTrajectoryTaskBuilder> {
        private Measure<Time> timeout = DEFAULT_TIMEOUT;
        private PriorityLevel priority = PriorityLevel.NORMAL;
        private String name = null;

        public RoadRunnerTrajectoryTaskBuilder(Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        }

        public RoadRunnerTrajectoryTaskBuilder(Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
            super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
        }

        /**
         * Set a timeout for the trajectory, to be applied to the overhead task running the trajectory.
         * Should be called first, before any other builder methods.
         * If this method is not called or fed negative values, an infinite timeout will be used.
         *
         * @param interval Timeout for the trajectory
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withTimeout(Measure<Time> interval) {
            if (interval.lt(INFINITE_TIMEOUT)) {
                return this;
            }
            timeout = interval;
            return this;
        }

        /**
         * Set the task name of the trajectory to show up in the telemetry.
         *
         * @param taskName Name of the task
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Set the priority level of the task.
         *
         * @param p Priority level
         * @return trajectory builder
         */
        public RoadRunnerTrajectoryTaskBuilder withPriority(PriorityLevel p) {
            priority = p;
            return this;
        }

        /**
         * Build the trajectory sequence and add it to the task queue with default priority.
         */
        @Override
        public TrajectorySequence build() {
            TrajectorySequence sequence = super.build();
            AddTrajectoryBuilder<TrajectorySequence> builder = new AddTrajectoryBuilder<>(sequence);
            builder.withTimeout(timeout)
                    .withPriority(priority)
                    .withName(name)
                    .build();
            return sequence;
        }
    }
}
