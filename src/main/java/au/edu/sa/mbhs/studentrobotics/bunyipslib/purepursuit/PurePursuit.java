package au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.exception.NotStrictlyPositiveException;

import java.util.ArrayList;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.AutonomousBunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.Path;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.purepursuit.path.PathBuilder;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.PurePursuitTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dashboard;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import kotlin.UninitializedPropertyAccessException;

/**
 * A parametric displacement-based implementation of a Pure Pursuit controller designed for holonomic drivetrains.
 * <p>
 * This system uses a series of suppliers and consumers to define odometry and drive power setting via
 * RoadRunner geometry utilities such as {@link Pose2d} along a {@link Path}. The actual lookahead, despite
 * being usually rooted in a line-circle intersection, is performed by pose projection based on looking ahead
 * on the path by displacement. By using displacement, the lookahead point naturally reduces for complex bends,
 * and centers the process of calculating the lookahead as a simple vector projection.
 * <p>
 * A RoadRunner drive instance is not required to use this class, the only RoadRunner utility used is the pose.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class PurePursuit implements Runnable {
    /**
     * Inches of how far apart guesses are made along the path for projecting the current robot pose on the path.
     * Lower values will negatively impact loop times but increase the path accuracy.
     */
    public static int PROJECTION_INTERVAL = 12;
    /**
     * Milliseconds of the interval the projections are updated.
     * Lower values will negatively impact loop times but increase the path accuracy.
     */
    public static int PROJECTION_REFRESH_RATE = 50;
    /**
     * Inches of how close the robot should be to the end of the path to stop the Pure Pursuit controller and
     * to simple move to PID To Point.
     */
    public static int P2P_AT_END_INCHES = 12;

    private final Moveable drive;
    private final ElapsedTime time = new ElapsedTime();

    private double pathProjection;
    private Pose2d lookahead;
    private PIDF p2pX;
    private PIDF p2pY;
    private PIDF p2pHeading;

    private Measure<Distance> tolerance;
    private Measure<Angle> angleTolerance;
    private Measure<Distance> laRadius;
    private Path currentPath = null;

    /**
     * Construct a new component to run Pure Pursuit pathing with.
     *
     * @param drive the drive instance that will be used for localisation and controlling motors
     */
    public PurePursuit(@NonNull Moveable drive) {
        this.drive = drive;

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

        FtcDashboard.getInstance().withConfigRoot(c ->
                c.putVariable(getClass().getSimpleName(), ReflectionConfig.createVariableFromClass(getClass())));
    }

    /**
     * Set the PIDF controller for the robot X axis of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the X axis
     * @return this
     */
    @NonNull
    public PurePursuit withXPIDF(@NonNull PIDF pidf) {
        p2pX = pidf;
        return this;
    }

    /**
     * Set the PIDF controller for the robot Y axis of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the Y axis
     * @return this
     */
    @NonNull
    public PurePursuit withYPIDF(@NonNull PIDF pidf) {
        p2pY = pidf;
        return this;
    }

    /**
     * Set the PIDF controller for the robot heading of the Pure Pursuit controller.
     *
     * @param pidf the PIDF controller to use for the heading
     * @return this
     */
    @NonNull
    public PurePursuit withHeadingPIDF(@NonNull PIDF pidf) {
        p2pHeading = pidf;
        return this;
    }

    /**
     * Set the distance tolerance that this Pure Pursuit controller will use to determine when to stop following a path.
     *
     * @param vectorDistance the distance tolerance for the path position
     * @return this
     */
    @NonNull
    public PurePursuit withVectorTolerance(@NonNull Measure<Distance> vectorDistance) {
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
    @NonNull
    public PurePursuit withHeadingTolerance(@NonNull Measure<Angle> headingDiff) {
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
     * @param headingDiff    the heading tolerance for the path heading
     * @return this
     */
    @NonNull
    public PurePursuit withTolerances(@NonNull Measure<Distance> vectorDistance, @NonNull Measure<Angle> headingDiff) {
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
    @NonNull
    public PurePursuit withLookaheadRadius(@NonNull Measure<Distance> lookaheadRadius) {
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
    @NonNull
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
        if (currentPath == null) {
            time.reset();
            return;
        }
        Pose2d currentPose = Objects.requireNonNull(drive.getPose(), "Non-null Localizer required for Pure Pursuit!");

        if (time.milliseconds() >= PROJECTION_REFRESH_RATE) {
            pathProjection = currentPath.project(currentPose.position, PROJECTION_INTERVAL);
            time.reset();
        }

        // This is where the actual lookahead is calculated, by parametrically "looking ahead" on the curve
        // to calculate the new vector that should be driven to. This will also ensure the path is always being followed,
        // as this is a parametric equation.
        lookahead = currentPath.get(pathProjection + laRadius.in(Inches));

        // Swap to P2P at the end of the path
        boolean isCloseToEnd = Geometry.distBetween(
                currentPath.get(pathProjection).position,
                currentPath.end().position
        ) < P2P_AT_END_INCHES;

        // Rotate vectors to field frame to ensure PID controllers are in the same frame of reference
        Vector2d rotCurr = currentPose.heading.inverse().times(currentPose.position);
        Vector2d rotLook = isCloseToEnd
                ? currentPose.heading.inverse().times(currentPath.end().position)
                : currentPose.heading.inverse().times(lookahead.position);

        // Wrap angle between -180 to 180 degrees, heading is calculated by projecting the current
        // pose onto the path to get the best target that the path is desiring near this point
        double headingError = Mathf.inputModulus(lookahead.heading.toDouble() - currentPose.heading.toDouble(), -Math.PI, Math.PI);
        if (Mathf.isNear(Math.abs(headingError), Math.PI, 0.1))
            headingError = Math.PI;

        drive.setPower(new PoseVelocity2d(
                new Vector2d(
                        Mathf.clamp(p2pX.calculate(rotCurr.x, rotLook.x), -1, 1),
                        Mathf.clamp(p2pY.calculate(rotCurr.y, rotLook.y), -1, 1)
                ),
                Mathf.clamp(-p2pHeading.calculate(headingError, 0), -1, 1)
        ));

        Dashboard.usePacket(p -> {
            Canvas canvas = p.fieldOverlay();

            // Path drawing
            canvas.setStroke("#7C4DFF");
            Dashboard.drawSampledPath(canvas, currentPath);
            // Circle at the end of the path
            Pose2d end = currentPath.end();
            canvas.setFill("#7C4DFF");
            canvas.fillCircle(end.position.x, end.position.y, 2);
            // Lookahead
            canvas.setFill("#4CAF50");
            canvas.setStroke("#4CAF50");
            canvas.strokeLine(currentPose.position.x, currentPose.position.y, lookahead.position.x, lookahead.position.y);
            canvas.fillCircle(lookahead.position.x, lookahead.position.y, 1.5);
            canvas.setStroke("#4CAF507F");
            canvas.strokeCircle(currentPose.position.x, currentPose.position.y, laRadius.in(Inches));
            // Lookahead path projection
            canvas.setStroke("#DD2C0076");
            canvas.setStrokeWidth(2);
            canvas.strokeCircle(lookahead.position.x, lookahead.position.y, 4);
            Vector2d v = lookahead.position.angleCast().vec();
            canvas.strokeLine(
                    lookahead.position.x + (v.x * 4) / 2,
                    lookahead.position.y + (v.y * 4) / 2,
                    lookahead.position.x + v.x * 4,
                    lookahead.position.y + v.y * 4
            );
            canvas.setStrokeWidth(1);
            // P2P circle
            canvas.setStroke("#4CAF507A");
            canvas.strokeCircle(end.position.x, end.position.y, P2P_AT_END_INCHES);
        });

        // Path position and heading tolerance finish condition check, which simply checks for the robot being close
        // to the end position. This is a simple check that is not perfect but is good enough for most cases, as
        // paths that go to the same point should probably be split into separate paths.
        // Note: intersecting paths don't work very nicely
        if (Geometry.distBetween(currentPose.position, currentPath.end().position) < tolerance.in(Inches) &&
                Mathf.isNear(
                        Mathf.inputModulus(currentPose.heading.toDouble(), -Math.PI, Math.PI),
                        Mathf.inputModulus(currentPath.end().heading.toDouble(), -Math.PI, Math.PI),
                        angleTolerance.in(Radians)
                )) {
            // Stop motors and release path
            drive.setPower(Geometry.zeroVel());
            currentPath = null;
        }
    }

    /**
     * Start constructing a new Pure Pursuit path that will start from the current drive pose which is captured
     * when the path is built (may be delegated if not using a task build method).
     *
     * @return the Pure Pursuit path and task builder
     */
    @NonNull
    public PathMaker makePath() {
        return makePath(drive::getPose);
    }

    /**
     * Start constructing a new Pure Pursuit path that starts from this static pose.
     *
     * @param staticStartPose the pose to start from
     * @return the Pure Pursuit path and task builder
     */
    @NonNull
    public PathMaker makePath(@NonNull Pose2d staticStartPose) {
        return makePath(() -> staticStartPose);
    }

    /**
     * Start constructing a new Pure Pursuit path that will start from the pose supplied by this
     * pose supplier when the path is built (may be delegated if not using a task build method).
     *
     * @param startPose the pose to use as the path starting pose when this path is started
     * @return the Pure Pursuit path and task builder
     */
    @NonNull
    public PathMaker makePath(@NonNull Supplier<Pose2d> startPose) {
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

        private Measure<Time> timeout = Task.INFINITE_TIMEOUT;
        private AutonomousBunyipsOpMode.TaskPriority priority = AutonomousBunyipsOpMode.TaskPriority.NORMAL;
        private String name = null;

        // Note: This path maker does not support path mirroring like the RoadRunner interface using a MirrorMap,
        // which may be added later if needed. At the moment, it is possible to mirror these paths manually.
        // Turning may be accomplished via a heading supplied interpolator or through the TurnTask.

        /**
         * Create a new PathMaker.
         *
         * @param startPoseAtRuntime the start pose to use when this path is requested
         */
        public PathMaker(@NonNull Supplier<Pose2d> startPoseAtRuntime) {
            startPose = startPoseAtRuntime;
            startTangent = () -> Radians.of(startPose.get().heading.toDouble());
        }

        /**
         * Run this path backwards by adding a 180-degree tangent to the start pose.
         *
         * @return this
         */
        @NonNull
        public PathMaker reversed() {
            startTangent = () -> Mathf.normaliseAngle(Radians.of(startPose.get().heading.toDouble() + Math.PI));
            return this;
        }

        /**
         * Run this path with this defined starting tangent.
         *
         * @param tangent the tangent to start with
         * @param unit    the unit of the tangent
         * @return this
         */
        @NonNull
        public PathMaker withStartTangent(double tangent, @NonNull Angle unit) {
            startTangent = () -> Mathf.normaliseAngle(unit.of(tangent));
            return this;
        }

        /**
         * Adds a line segment with tangent heading interpolation.
         *
         * @param endPosition the end position
         * @param unit        units of the supplied vector
         * @return this
         */
        @NonNull
        public PathMaker lineTo(@NonNull Vector2d endPosition, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.lineTo(new Vector2d(
                            unit.of(endPosition.x).in(Inches),
                            unit.of(endPosition.y).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with constant heading interpolation.
         *
         * @param endPosition the end position
         * @param unit        units of the supplied vector
         * @return this
         */
        @NonNull
        public PathMaker lineToConstantHeading(@NonNull Vector2d endPosition, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.lineToConstantHeading(new Vector2d(
                            unit.of(endPosition.x).in(Inches),
                            unit.of(endPosition.y).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
         *
         * @param endPosition the end position
         * @param unit        units of the supplied vector
         * @return this
         */
        @NonNull
        public PathMaker strafeTo(@NonNull Vector2d endPosition, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeTo(new Vector2d(
                            unit.of(endPosition.x).in(Inches),
                            unit.of(endPosition.y).in(Inches)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with linear heading interpolation.
         *
         * @param endVec     the end vector
         * @param vectorUnit units of the supplied vector
         * @param angle      the angle to linear interpolate to
         * @param angleUnit  units of the supplied angle
         * @return this
         */
        @NonNull
        public PathMaker lineToLinearHeading(@NonNull Vector2d endVec, @NonNull Distance vectorUnit, double angle, @NonNull Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.lineToLinearHeading(new Pose2d(
                            vectorUnit.of(endVec.x).in(Inches),
                            vectorUnit.of(endVec.y).in(Inches),
                            angleUnit.of(angle).in(Radians)
                    ))
            );
            return this;
        }

        /**
         * Adds a line segment with spline heading interpolation.
         *
         * @param endVec     the end vector
         * @param vectorUnit units of the supplied vector
         * @param angle      the angle to spline interpolate to
         * @param angleUnit  units of the supplied angle
         * @return this
         */
        @NonNull
        public PathMaker lineToSplineHeading(@NonNull Vector2d endVec, @NonNull Distance vectorUnit, double angle, @NonNull Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.lineToSplineHeading(new Pose2d(
                            vectorUnit.of(endVec.x).in(Inches),
                            vectorUnit.of(endVec.y).in(Inches),
                            angleUnit.of(angle).in(Radians)
                    ))
            );
            return this;
        }

        /**
         * Adds a line straight forward.
         *
         * @param distance the distance to travel forward
         * @param unit     the unit of the distance
         * @return this
         */
        @NonNull
        public PathMaker forward(double distance, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.forward(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a line straight backward.
         *
         * @param distance the distance to travel backward
         * @param unit     the unit of the distance
         * @return this
         */
        @NonNull
        public PathMaker back(double distance, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.back(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a segment that strafes left in the robot reference frame.
         *
         * @param distance the distance to strafe left
         * @param unit     the unit of the distance
         * @return this
         */
        @NonNull
        public PathMaker strafeLeft(double distance, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeLeft(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a segment that strafes right in the robot reference frame.
         *
         * @param distance the distance to strafe right
         * @param unit     the unit of the distance
         * @return this
         */
        @NonNull
        public PathMaker strafeRight(double distance, @NonNull Distance unit) {
            buildInstructions.add((b) ->
                    b.strafeRight(unit.of(distance).in(Inches))
            );
            return this;
        }

        /**
         * Adds a spline segment with tangent heading interpolation.
         *
         * @param endPosition  the end position
         * @param distanceUnit units of the supplied vector
         * @param endTangent   the end tangent
         * @param angleUnit    units of the supplied angle
         * @return this
         */
        @NonNull
        public PathMaker splineTo(@NonNull Vector2d endPosition, @NonNull Distance distanceUnit, double endTangent, @NonNull Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.splineTo(new Vector2d(
                            distanceUnit.of(endPosition.x).in(Inches),
                            distanceUnit.of(endPosition.y).in(Inches)
                    ), angleUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with constant heading interpolation.
         *
         * @param endPosition  the end position
         * @param distanceUnit units of the supplied vector
         * @param endTangent   the end tangent
         * @param angleUnit    units of the supplied angle
         * @return this
         */
        @NonNull
        public PathMaker splineToConstantHeading(@NonNull Vector2d endPosition, @NonNull Distance distanceUnit, double endTangent, @NonNull Angle angleUnit) {
            buildInstructions.add((b) ->
                    b.splineToConstantHeading(new Vector2d(
                            distanceUnit.of(endPosition.x).in(Inches),
                            distanceUnit.of(endPosition.y).in(Inches)
                    ), angleUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with linear heading interpolation.
         *
         * @param endVec         the end vector
         * @param vectorUnit     units of the supplied vector
         * @param angle          the angle to linear interpolate to
         * @param angleUnit      units of the supplied angle in the end pose
         * @param endTangent     the end tangent
         * @param endTangentUnit units of the supplied angle for the end tangent
         * @return this
         */
        @NonNull
        public PathMaker splineToLinearHeading(@NonNull Vector2d endVec, @NonNull Distance vectorUnit, double angle, @NonNull Angle angleUnit, double endTangent, @NonNull Angle endTangentUnit) {
            buildInstructions.add((b) ->
                    b.splineToLinearHeading(new Pose2d(
                            vectorUnit.of(endVec.x).in(Inches),
                            vectorUnit.of(endVec.y).in(Inches),
                            angleUnit.of(angle).in(Radians)
                    ), endTangentUnit.of(endTangent).in(Radians))
            );
            return this;
        }

        /**
         * Adds a spline segment with spline heading interpolation.
         *
         * @param endVec         the end vector
         * @param vectorUnit     units of the supplied vector
         * @param angle          the angle to linear interpolate to
         * @param angleUnit      units of the supplied angle in the end pose
         * @param endTangent     the end tangent
         * @param endTangentUnit units of the supplied angle for the end tangent
         * @return this
         */
        @NonNull
        public PathMaker splineToSplineHeading(@NonNull Vector2d endVec, @NonNull Distance vectorUnit, double angle, @NonNull Angle angleUnit, double endTangent, @NonNull Angle endTangentUnit) {
            buildInstructions.add((b) ->
                    b.splineToSplineHeading(new Pose2d(
                            vectorUnit.of(endVec.x).in(Inches),
                            vectorUnit.of(endVec.y).in(Inches),
                            angleUnit.of(angle).in(Radians)
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
        @NonNull
        public PathMaker withTimeout(@NonNull Measure<Time> interval) {
            if (interval.lt(Task.INFINITE_TIMEOUT)) {
                timeout = Task.INFINITE_TIMEOUT;
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
        @NonNull
        public PathMaker withName(@NonNull String taskName) {
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
        @NonNull
        public PathMaker withPriority(@NonNull AutonomousBunyipsOpMode.TaskPriority p) {
            priority = p;
            return this;
        }

        /**
         * Builds the path and returns it. This will evaluate the path at the current pose if it is a supplier,
         * which is equivalent to the path being built at runtime.
         *
         * @return the built path from the instructions
         */
        @NonNull
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
        @NonNull
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
        @NonNull
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
         * @return the task instance to allow for {@code onSubsystem} chaining
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        @NonNull
        public PurePursuitTask addTask() throws UninitializedPropertyAccessException {
            if (!BunyipsOpMode.isRunning() || !(BunyipsOpMode.getInstance() instanceof AutonomousBunyipsOpMode)) {
                throw new UninitializedPropertyAccessException("Cannot call addTask() when there is no running AutonomousBunyipsOpMode!");
            }
            PurePursuitTask task = buildTask();
            ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTask(priority, task);
            return task;
        }

        /**
         * Adds a task to the {@link AutonomousBunyipsOpMode} queue that builds the path now and returns a task that will
         * track the path when executed.
         * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
         * on construction, and needs to be done manually via the {@code onSubsystem} method.
         *
         * @return the task instance to allow for {@code onSubsystem} chaining
         * @throws UninitializedPropertyAccessException if the task is added outside of an {@link AutonomousBunyipsOpMode}
         */
        @NonNull
        public PurePursuitTask addTaskNow() throws UninitializedPropertyAccessException {
            if (!BunyipsOpMode.isRunning() || !(BunyipsOpMode.getInstance() instanceof AutonomousBunyipsOpMode)) {
                throw new UninitializedPropertyAccessException("Cannot call addTaskNow() when there is no running AutonomousBunyipsOpMode!");
            }
            PurePursuitTask task = buildTaskNow();
            ((AutonomousBunyipsOpMode) BunyipsOpMode.getInstance()).addTask(priority, task);
            return task;
        }
    }
}
