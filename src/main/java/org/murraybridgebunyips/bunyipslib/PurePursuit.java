package org.murraybridgebunyips.bunyipslib;


import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import android.util.Pair;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.apache.commons.math3.exception.NotStrictlyPositiveException;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.PIDF;
import org.murraybridgebunyips.bunyipslib.external.pid.PController;
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
 * Simple implementation of a Pure Pursuit controller designed for holonomic drivetrains.
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
@Config
public class PurePursuit implements Runnable {
    /**
     * Inches of how close together line segments should be to find the best lookahead point.
     */
    public static final int SEGMENTATION_APPROXIMATION_STEP = 4;
    /**
     * Inches of how close the lookahead point approximations should be to determine the best lookahead point.
     */
    public static final int BEST_LOOKAHEAD_POINT_GUESS_STEP = 6;
    /**
     * Inches of how close the heading projection approximations should be to determine the closest point on the path.
     */
    public static final int HEADING_PROJECTION_GUESS_STEP = 2;
    /**
     * Inches of how close the robot should be to the end of the path to stop the Pure Pursuit controller and
     * to simple move to PID To Point.
     */
    public static final int P2P_AT_END_INCHES = 10;

    private final Consumer<Pose2d> power;
    private final Supplier<Pose2d> pose;

    private PIDF p2pX;
    private PIDF p2pY;
    private PIDF p2pHeading;

    private Measure<Distance> tolerance;
    private Measure<Angle> angleTolerance;
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
        p2pX = new PController(6);
        p2pY = new PController(6);
        p2pHeading = new PController(3);
        tolerance = Inches.of(1);
        angleTolerance = Degrees.of(1);

        BunyipsOpMode.ifRunning(opMode -> {
            opMode.onActiveLoop(this);
            Dbg.logd(getClass(), "Update executor has been auto-attached to BunyipsOpMode.");
        });
    }

    /**
     * Construct a new component to run Pure Pursuit pathing with.
     *
     * @param drive the RoadRunner drive instance that will be used for localisation and controlling motors,
     *              note that the trajectory system from RoadRunner is not utilised from this drive
     */
    public PurePursuit(RoadRunnerDrive drive) {
        this(drive::setRotationPriorityWeightedDrivePower, drive::getPoseEstimate);
        // We can auto extract PID coefficients from the drive instance if it is a MecanumDrive or TankDrive
        if (drive instanceof MecanumDrive) {
            MecanumDrive mecanumDrive = (MecanumDrive) drive;
            PIDCoefficients xyCoeffs = mecanumDrive.getCoefficients().TRANSLATIONAL_PID;
            PIDCoefficients rCoeffs = mecanumDrive.getCoefficients().HEADING_PID;
            p2pX.getPIDFController().setPIDF(xyCoeffs.kP, xyCoeffs.kI, xyCoeffs.kD, 0);
            p2pY.getPIDFController().setPIDF(xyCoeffs.kP, xyCoeffs.kI, xyCoeffs.kD, 0);
            p2pHeading.getPIDFController().setPIDF(rCoeffs.kP, rCoeffs.kI, rCoeffs.kD, 0);
        } else if (drive instanceof TankDrive) {
            TankDrive tankDrive = (TankDrive) drive;
            PIDCoefficients xCoeffs = tankDrive.getCoefficients().AXIAL_PID;
            PIDCoefficients yCoeffs = tankDrive.getCoefficients().CROSS_TRACK_PID;
            PIDCoefficients rCoeffs = tankDrive.getCoefficients().HEADING_PID;
            p2pX.getPIDFController().setPIDF(xCoeffs.kP, xCoeffs.kI, xCoeffs.kD, 0);
            p2pY.getPIDFController().setPIDF(yCoeffs.kP, yCoeffs.kI, yCoeffs.kD, 0);
            p2pHeading.getPIDFController().setPIDF(rCoeffs.kP, rCoeffs.kI, rCoeffs.kD, 0);
        }
    }

    /**
     * Set the PIDF controller for the robot X axis of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the X axis
     * @return this
     */
    public PurePursuit withXPIDF(PIDF pidf) {
        p2pX = pidf;
        return this;
    }

    /**
     * Set the PIDF controller for the robot Y axis of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the Y axis
     * @return this
     */
    public PurePursuit withYPIDF(PIDF pidf) {
        p2pY = pidf;
        return this;
    }

    /**
     * Set the PIDF controller for the robot heading of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the heading
     * @return this
     */
    public PurePursuit withHeadingPIDF(PIDF pidf) {
        p2pHeading = pidf;
        return this;
    }

    /**
     * Set the distance tolerance that this Pure Pursuit controller will use to determine when to stop following a path.
     *
     * @param vectorDistance the distance tolerance for the path position
     * @return this
     */
    public PurePursuit withVectorTolerance(Measure<Distance> vectorDistance) {
        if (vectorDistance.magnitude() <= 0) {
            throw new NotStrictlyPositiveException(vectorDistance.magnitude());
        }
        tolerance = vectorDistance;
        return this;
    }

    /**
     * Set the heading tolerance that this Pure Pursuit controller will use to determine when to stop following a path.
     *
     * @param headingDiff the heading tolerance for the path heading
     * @return this
     */
    public PurePursuit withHeadingTolerance(Measure<Angle> headingDiff) {
        if (headingDiff.magnitude() <= 0) {
            throw new NotStrictlyPositiveException(headingDiff.magnitude());
        }
        angleTolerance = headingDiff;
        return this;
    }

    /**
     * Set the tolerances that this Pure Pursuit controller will use to determine when to stop following a path.
     *
     * @param vectorDistance the distance tolerance for the path position
     * @param headingDiff the heading tolerance for the path heading
     * @return this
     */
    public PurePursuit withTolerances(Measure<Distance> vectorDistance, Measure<Angle> headingDiff) {
        withVectorTolerance(vectorDistance);
        withHeadingTolerance(headingDiff);
        return this;
    }

    /**
     * Set a new lookahead radius that will be used for current and future paths.
     *
     * @param lookaheadRadius the lookahead radius to use
     * @return this
     */
    public PurePursuit withLookaheadRadius(Measure<Distance> lookaheadRadius) {
        if (lookaheadRadius.magnitude() <= 0) {
            throw new NotStrictlyPositiveException(lookaheadRadius.magnitude());
        }
        laRadius = lookaheadRadius;
        return this;
    }

    /**
     * Set a path that will be followed during the execution of this Pure Pursuit controller.
     *
     * @param path the path to follow with the current lookahead radius
     */
    public void followPath(@NonNull Path path) {
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
        Pose2d poseEstimate = pose.get();

        Pose2d targetPower = runPurePursuit(poseEstimate);
        power.accept(targetPower);

        // Path position and heading tolerance finish condition check, which simply checks for the robot being close
        // to the end position. This is a simple check that is not perfect but is good enough for most cases, as
        // paths that go to the same point should probably be split into separate paths.
        if (poseEstimate.vec().distTo(currentPath.end().vec()) < tolerance.in(Inches) &&
                Mathf.isNear(poseEstimate.getHeading(), currentPath.end().getHeading(), angleTolerance.in(Radians))) {
            // Stop motors and release path
            power.accept(new Pose2d());
            currentPath = null;
        }
    }

    private Pose2d runPurePursuit(Pose2d currentPose) {
        Vector2d lookahead = null;

        for (int i = 0; i < currentPath.length(); i += SEGMENTATION_APPROXIMATION_STEP) {
            // Using a line segment which traces from approximately small segments of the path to find the best lookahead
            Vector2d start = currentPath.get(i).vec();
            Vector2d end = currentPath.get(i + SEGMENTATION_APPROXIMATION_STEP).vec();
            try {
                // We now apply the lookahead radius to get the 2 solutions from the quadratic
                Pair<Vector2d, Vector2d> intersections = Mathf.lineSegmentCircleIntersection(
                        start, end, currentPose.vec(), laRadius.in(Inches)
                );
                // We then project the intersections onto the path to get the best lookahead point, as one will
                // likely be behind the robot (not progressed on the path)
                double s1 = currentPath.project(intersections.first, BEST_LOOKAHEAD_POINT_GUESS_STEP);
                double s2 = currentPath.project(intersections.second, BEST_LOOKAHEAD_POINT_GUESS_STEP);
                // Maximisation of the parameter to get the best lookahead point
                if (s1 > s2) {
                    lookahead = intersections.first;
                } else {
                    lookahead = intersections.second;
                }
                break;
            } catch (Mathf.NoInterceptException e) {
                // No intercept, continue to next segment
            }
        }

        Pose2d pathProjection = currentPath.get(currentPath.project(currentPose.vec(), HEADING_PROJECTION_GUESS_STEP));
        if (lookahead == null) {
            // Travel to the closest projection of the path if the circle has no intercepts anywhere along the path
            lookahead = pathProjection.vec();
        }

        // Swap to P2P at the end of the path
        boolean isCloseToEnd = pathProjection.vec().distTo(currentPath.end().vec()) < P2P_AT_END_INCHES;

        // Rotate vectors to field frame to ensure PID controllers are in the same frame of reference
        Vector2d rotCurr = currentPose.vec().rotated(-currentPose.getHeading());
        Vector2d rotLook = isCloseToEnd ? currentPath.end().vec().rotated(-currentPose.getHeading()) : lookahead.rotated(-currentPose.getHeading());

        // Wrap angle between -180 to 180 degrees, heading is calculated by projecting the current
        // pose onto the path to get the best target that the path is desiring near this point
        double targetHeading = Mathf.inputModulus(pathProjection.getHeading(), -Math.PI, Math.PI);
        // The heading at the modulus boundary acts strangely, so we'll just lock it when it gets close
        if (Mathf.approximatelyEquals(Math.abs(currentPath.end().getHeading()), Math.PI) && isCloseToEnd)
            targetHeading = Math.PI;

        return new Pose2d(
                p2pX.calculate(rotCurr.getX(), rotLook.getX()),
                p2pY.calculate(rotCurr.getY(), rotLook.getY()),
                p2pHeading.calculate(Mathf.inputModulus(currentPose.getHeading(), -Math.PI, Math.PI), targetHeading)
        );
    }

    /**
     * Start constructing a new Pure Pursuit path that will start from the current drive pose which is captured
     * when the path is built (may be delegated if not using a task build method).
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
     * pose supplier when the path is built (may be delegated if not using a task build method).
     *
     * @param startPose the pose to use as the path starting pose when this path is started
     * @return the Pure Pursuit path and task builder
     */
    public PathMaker makePath(Supplier<Pose2d> startPose) {
        return new PathMaker(startPose);
    }

    /**
     * Utility construction class for Path instances that can be executed by {@link PurePursuit}.
     * Pathing generated through this builder must follow continuity.
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
        // Turning may be accomplished via a heading supplied interpolator or through the TurnTask.

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
         * Builds the path and returns it. This will evaluate the path at the current pose if it is a supplier,
         * which is equivalent to the path being built at runtime.
         *
         * @return the built path from the instructions
         */
        public Path buildPath() {
            PathBuilder builder = new PathBuilder(startPose.get(), startTangent.get().in(Radians));
            buildInstructions.forEach(instruction -> instruction.accept(builder));
            return builder.build();
        }

        /**
         * Build a task that will build and run the path when executed.
         * This is similar to a deferred task but applies to the path construction.
         * <p>
         * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
         * on construction, and needs to be done manually via the {@code onSubsystem} method.
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
         * <p>
         * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
         * on construction, and needs to be done manually via the {@code onSubsystem} method.
         *
         * @return the task that will track the built path when executed
         */
        public PurePursuitTask buildTaskNow() {
            PurePursuitTask task = new PurePursuitTask(PurePursuit.this, buildPath());
            task.withName(name);
            task.withTimeout(timeout);
            return task;
        }

        /**
         * Adds a task to the {@link AutonomousBunyipsOpMode} queue that will build and run the path when executed.
         * This is similar to a deferred task but applies to the path construction.
         * <p>
         * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
         * on construction, and needs to be done manually via the {@code onSubsystem} method.
         *
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        public PurePursuitTask addTask() throws UninitializedPropertyAccessException {
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
            return task;
        }

        /**
         * Adds a task to the {@link AutonomousBunyipsOpMode} queue that builds the path now and returns a task that will
         * track the path when executed.
         * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
         * on construction, and needs to be done manually via the {@code onSubsystem} method.
         *
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        public PurePursuitTask addTaskNow() throws UninitializedPropertyAccessException {
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
            return task;
        }
    }
}
