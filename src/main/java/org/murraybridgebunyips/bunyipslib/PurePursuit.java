package org.murraybridgebunyips.bunyipslib;


import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.PurePursuitTask;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import kotlin.UninitializedPropertyAccessException;

/**
 * Component implementation of a Pure Pursuit controller designed for holonomic drivetrains.
 * <p>
 * This system uses a series of suppliers and consumers to define odometry and drive power setting via
 * RoadRunner geometry utilities such as {@link Pose2d} along a {@link Path}.
 * <p>
 * A RoadRunner drive instance is not required to use this class, but since it provides odometry it is designed
 * to support consuming a {@link RoadRunnerDrive} instance if wanted.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class PurePursuit implements Runnable {
    /**
     * Internal flag to determine if this Pure Pursuit controller has been auto-attached
     * to the BunyipsOpMode active loop.
     */
    public final boolean ATTACHED_TO_BOM_LOOP;

    private final Consumer<Pose2d> power;
    private final Supplier<Pose2d> pose;

    private Measure<Distance> laRadius;
    private Path currentPath = null;

    /**
     * Construct a new component to run Pure Pursuit pathing with using your own
     * odometry implementation and drive powers.
     *
     * @param setDrivePower a consumer that will take in a pose the motors should be commanded to move in during a path execution
     * @param poseEstimateSupplier a supplier that supplies the current robot pose as per your odometry
     */
    public PurePursuit(Consumer<Pose2d> setDrivePower, Supplier<Pose2d> poseEstimateSupplier) {
        power = setDrivePower;
        pose = poseEstimateSupplier;

        // Sane defaults
        laRadius = Inches.of(18);

        ATTACHED_TO_BOM_LOOP = BunyipsOpMode.isRunning();
        BunyipsOpMode.ifRunning(opMode -> {
            opMode.onActiveLoop(this);
            Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode");
        });
    }

    /**
     * Construct a new component to run Pure Pursuit pathing with.
     *
     * @param drive the RoadRunner drive instance that will be used for localisation and controlling motors,
     *              note that the trajectory system from RoadRunner is not utilised from this drive
     */
    public PurePursuit(RoadRunnerDrive drive) {
        this(drive::setWeightedDrivePower, drive::getPoseEstimate);
    }

    /**
     * Set a new lookahead radius that will be used for current and future paths.
     *
     * @param lookaheadRadius the lookahead radius to use
     * @return this
     */
    public PurePursuit withLookaheadRadius(Measure<Distance> lookaheadRadius) {
        laRadius = lookaheadRadius;
        return this;
    }

    /**
     * Set a path that will be followed during the execution of this Pure Pursuit controller.
     *
     * @param path the path to follow with the current lookahead radius
     */
    public void followPath(Path path) {
        currentPath = path;
    }

    /**
     * @return the current path that is being followed
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    /**
     * Clear and terminate the currently running path.
     */
    public void clearPath() {
        currentPath = null;
    }

    /**
     * @return whether a path is currently set for this controller
     */
    public boolean isBusy() {
        return currentPath != null;
    }

    /**
     * Commands an iteration of the Pure Pursuit controller for the current path.
     * Will no-op if there is no path to run.
     */
    @Override
    public void run() {
        if (currentPath == null) return;

        // TODO
    }

    /**
     * Start constructing a new Pure Pursuit path that will start from the current drive pose which is captured
     * when the path is built (may be delegated if not using a {@code now} build method).
     *
     * @return the Pure Pursuit path and task builder
     */
    public PathMaker makePath() {
        return makePath(pose);
    }

    /**
     * Start constructing a new Pure Pursuit path that starts from this static pose.
     *
     * @param staticStartPose the pose to start from
     * @return the Pure Pursuit path and task builder
     */
    public PathMaker makePath(Pose2d staticStartPose) {
        return makePath(() -> staticStartPose);
    }

    /**
     * Start constructing a new Pure Pursuit path that will start from the pose supplied by this
     * pose supplier when the path is built (may be delegated if not using a {@code now} build method).
     *
     * @param startPose the pose to use as the path starting pose when this path is started
     * @return the Pure Pursuit path and task builder
     */
    public PathMaker makePath(Supplier<Pose2d> startPose) {
        return new PathMaker(startPose);
    }

    /**
     * Utility construction class for Path instances that can be executed by {@link PurePursuit}.
     */
    public class PathMaker {
        private final ArrayList<Consumer<PathBuilder>> buildInstructions = new ArrayList<>();
        private final Supplier<Pose2d> startPose;
        private Supplier<Measure<Angle>> startTangent;

        private Measure<Time> timeout = INFINITE_TIMEOUT;
        private RoadRunner.PriorityLevel priority = RoadRunner.PriorityLevel.NORMAL;
        private String name = null;

        // Note: This path maker does not support path mirroring like the RoadRunner interface using a MirrorMap,
        // which may be added later if needed. At the moment, it is possible to mirror these paths manually.

        /**
         * Create a new PathMaker.
         *
         * @param startPoseAtRuntime the start pose to use when this path is requested
         */
        public PathMaker(Supplier<Pose2d> startPoseAtRuntime) {
            startPose = startPoseAtRuntime;
            startTangent = () -> Radians.of(startPose.get().getHeading());
        }

        /**
         * Run this path backwards by adding a 180-degree tangent to the start pose.
         *
         * @return this
         */
        public PathMaker reversed() {
            startTangent = () -> Mathf.normaliseAngle(Radians.of(startPose.get().getHeading() + Math.PI));
            return this;
        }

        /**
         * Run this path with this defined starting tangent.
         *
         * @param tangent the tangent to start with
         * @param unit the unit of the tangent
         * @return this
         */
        public PathMaker withStartTangent(double tangent, Angle unit) {
            startTangent = () -> Mathf.normaliseAngle(unit.of(tangent));
            return this;
        }

        /**
         * Adds a line segment with tangent heading interpolation.
         *
         * @param endPosition the end position
         * @param unit units of the supplied vector
         * @return this
         */
        public PathMaker lineTo(Vector2d endPosition, Distance unit) {
            buildInstructions.add((b) ->
                    b.lineTo(new Vector2d(
                            unit.of(endPosition.getX()).in(Inches),
                            unit.of(endPosition.getY()).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with constant heading interpolation.
         *
         * @param endPosition the end position
         * @param unit units of the supplied vector
         * @return this
         */
        public PathMaker lineToConstantHeading(Vector2d endPosition, Distance unit) {
            buildInstructions.add((b) ->
                    b.lineToConstantHeading(new Vector2d(
                            unit.of(endPosition.getX()).in(Inches),
                            unit.of(endPosition.getY()).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
         *
         * @param endPosition the end position
         * @param unit units of the supplied vector
         * @return this
         */
        public PathMaker strafeTo(Vector2d endPosition, Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeTo(new Vector2d(
                            unit.of(endPosition.getX()).in(Inches),
                            unit.of(endPosition.getY()).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with linear heading interpolation.
         *
         * @param endPose the end pose
         * @param vectorUnit units of the supplied vector
         * @param angleUnit units of the supplied angle
         * @return this
         */
        public PathMaker lineToLinearHeading(Pose2d endPose, Distance vectorUnit, Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.lineToLinearHeading(new Pose2d(
                            vectorUnit.of(endPose.getX()).in(Inches),
                            vectorUnit.of(endPose.getY()).in(Inches),
                            angleUnit.of(endPose.getHeading()).in(Radians)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with spline heading interpolation.
         *
         * @param endPose the end pose
         * @param vectorUnit units of the supplied vector
         * @param angleUnit units of the supplied angle
         * @return this
         */
        public PathMaker lineToSplineHeading(Pose2d endPose, Distance vectorUnit, Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.lineToSplineHeading(new Pose2d(
                            vectorUnit.of(endPose.getX()).in(Inches),
                            vectorUnit.of(endPose.getY()).in(Inches),
                            angleUnit.of(endPose.getHeading()).in(Radians)
                    ))
            );
            return this;
        }

        /**
         * Adds a line straight forward.
         *
         * @param distance the distance to travel forward
         * @param unit the unit of the distance
         * @return this
         */
        public PathMaker forward(double distance, Distance unit) {
            buildInstructions.add((b) ->
                    b.forward(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a line straight backward.
         *
         * @param distance the distance to travel backward
         * @param unit the unit of the distance
         * @return this
         */
        public PathMaker back(double distance, Distance unit) {
            buildInstructions.add((b) ->
                    b.back(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a segment that strafes left in the robot reference frame.
         *
         * @param distance the distance to strafe left
         * @param unit the unit of the distance
         * @return this
         */
        public PathMaker strafeLeft(double distance, Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeLeft(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a segment that strafes right in the robot reference frame.
         *
         * @param distance the distance to strafe right
         * @param unit the unit of the distance
         * @return this
         */
        public PathMaker strafeRight(double distance, Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeRight(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a spline segment with tangent heading interpolation.
         *
         * @param endPosition the end position
         * @param distanceUnit units of the supplied vector
         * @param endTangent the end tangent
         * @param angleUnit units of the supplied angle
         * @return this
         */
        public PathMaker splineTo(Vector2d endPosition, Distance distanceUnit, double endTangent, Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.splineTo(new Vector2d(
                            distanceUnit.of(endPosition.getX()).in(Inches),
                            distanceUnit.of(endPosition.getY()).in(Inches)
                    ), angleUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with constant heading interpolation.
         *
         * @param endPosition the end position
         * @param distanceUnit units of the supplied vector
         * @param endTangent the end tangent
         * @param angleUnit units of the supplied angle
         * @return this
         */
        public PathMaker splineToConstantHeading(Vector2d endPosition, Distance distanceUnit, double endTangent, Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.splineToConstantHeading(new Vector2d(
                            distanceUnit.of(endPosition.getX()).in(Inches),
                            distanceUnit.of(endPosition.getY()).in(Inches)
                    ), angleUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with linear heading interpolation.
         *
         * @param endPose the end pose
         * @param vectorUnit units of the supplied vector in the end pose
         * @param angleUnit units of the supplied angle in the end pose
         * @param endTangent the end tangent
         * @param endTangentUnit units of the supplied angle for the end tangent
         * @return this
         */
        public PathMaker splineToLinearHeading(Pose2d endPose, Distance vectorUnit, Angle angleUnit, double endTangent, Angle endTangentUnit) {
            buildInstructions.add((b) ->
                    b.splineToLinearHeading(new Pose2d(
                            vectorUnit.of(endPose.getX()).in(Inches),
                            vectorUnit.of(endPose.getY()).in(Inches),
                            angleUnit.of(endPose.getHeading()).in(Radians)
                    ), endTangentUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with spline heading interpolation.
         *
         * @param endPose the end pose
         * @param vectorUnit units of the supplied vector in the end pose
         * @param angleUnit units of the supplied angle in the end pose
         * @param endTangent the end tangent
         * @param endTangentUnit units of the supplied angle for the end tangent
         * @return this
         */
        public PathMaker splineToSplineHeading(Pose2d endPose, Distance vectorUnit, Angle angleUnit, double endTangent, Angle endTangentUnit) {
            buildInstructions.add((b) ->
                    b.splineToSplineHeading(new Pose2d(
                            vectorUnit.of(endPose.getX()).in(Inches),
                            vectorUnit.of(endPose.getY()).in(Inches),
                            angleUnit.of(endPose.getHeading()).in(Radians)
                    ), endTangentUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Set a timeout for the path, to be applied to the overhead task running the path.
         * If this method is not called or fed negative values, an infinite timeout will be used.
         *
         * @param interval Timeout for the path task built through this builder
         * @return this
         */
        public PathMaker withTimeout(Measure<Time> interval) {
            if (interval.lt(INFINITE_TIMEOUT)) {
                timeout = INFINITE_TIMEOUT;
                return this;
            }
            timeout = interval;
            return this;
        }

        /**
         * Set a custom task name of the path to show up in the telemetry.
         *
         * @param taskName Name of the task
         * @return this
         */
        public PathMaker withName(String taskName) {
            name = taskName;
            return this;
        }

        /**
         * Set the priority level of the task if being added to a {@link AutonomousBunyipsOpMode} via the add task
         * methods.
         *
         * @param p Priority level
         * @return this
         */
        public PathMaker withPriority(RoadRunner.PriorityLevel p) {
            // Using the RoadRunner task priority level as it works the same anyways
            priority = p;
            return this;
        }

        /**
         * Builds the path and returns it.
         *
         * @return the built path from the instructions
         */
        public Path buildPathNow() {
            PathBuilder builder = new PathBuilder(startPose.get(), startTangent.get().in(Radians));
            buildInstructions.forEach(instruction -> instruction.accept(builder));
            return builder.build();
        }

        /**
         * Build a task that will build and run the path when executed.
         * This is similar to a deferred task but applies to the path construction.
         *
         * @return the task that will build and run the path when executed
         */
        public PurePursuitTask buildTask() {
            PurePursuitTask task = new PurePursuitTask(PurePursuit.this, this);
            task.withName(name);
            task.withTimeout(timeout);
            return task;
        }

        /**
         * Builds the path now and returns a task that will track the path when executed.
         *
         * @return the task that will track the built path when executed
         */
        public PurePursuitTask buildTaskNow() {
            PurePursuitTask task = new PurePursuitTask(PurePursuit.this, buildPathNow());
            task.withName(name);
            task.withTimeout(timeout);
            return task;
        }

        /**
         * Adds a task to the {@link AutonomousBunyipsOpMode} queue that will build and run the path when executed.
         * This is similar to a deferred task but applies to the path construction.
         *
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        public void addTask() throws UninitializedPropertyAccessException {
            if (!BunyipsOpMode.isRunning() || !(BunyipsOpMode.getInstance() instanceof AutonomousBunyipsOpMode)) {
                throw new UninitializedPropertyAccessException("Cannot call addTask() when there is no running AutonomousBunyipsOpMode!");
            }
            PurePursuitTask task = buildTask();
            switch (priority) {
                case LAST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskLast(task);
                    break;
                case NORMAL:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTask(task);
                    break;
                case FIRST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskFirst(task);
                    break;
            }
        }

        /**
         * Adds a task to the {@link AutonomousBunyipsOpMode} queue that builds the path now and returns a task that will
         * track the path when executed.
         *
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        public void addTaskNow() throws UninitializedPropertyAccessException {
            if (!BunyipsOpMode.isRunning() || !(BunyipsOpMode.getInstance() instanceof AutonomousBunyipsOpMode)) {
                throw new UninitializedPropertyAccessException("Cannot call addTaskNow() when there is no running AutonomousBunyipsOpMode!");
            }
            PurePursuitTask task = buildTaskNow();
            switch (priority) {
                case LAST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskLast(task);
                    break;
                case NORMAL:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTask(task);
                    break;
                case FIRST:
                    ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTaskFirst(task);
                    break;
            }
        }
    }
}
