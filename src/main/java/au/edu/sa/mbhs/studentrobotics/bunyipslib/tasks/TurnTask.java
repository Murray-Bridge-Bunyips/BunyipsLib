package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Degrees;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Radians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.apache.commons.math3.exception.NotStrictlyPositiveException;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Drawing;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * A simple task that turns the robot to a specific angle using a PIDF controller.
 *
 * @author Lucas Bubner, 2024
 * @see DriveToPoseTask
 * @since 5.1.0
 */
public class TurnTask extends Task {
    private final Consumer<PoseVelocity2d> powerIn;
    private final Supplier<Pose2d> poseEstimate;
    private final boolean setDelta;
    private final double unmodifiedAngRad;
    private double angRad;
    private PIDF pidf;
    private Measure<Angle> tolerance;

    /**
     * Construct a new TurnTask that will turn the robot to the given angle.
     *
     * @param drive the drive instance to use, this subsystem will be automatically attached to the task
     *              if it is a {@link BunyipsSubsystem}
     * @param angle the angle to turn to, if this is a delta angle this will be counter-clockwise
     * @param delta if this angle is a delta from the current drive rotation at runtime
     */
    public TurnTask(Moveable drive, Measure<Angle> angle, boolean delta) {
        this(drive::setPower, () -> Objects.requireNonNull(drive.getPoseEstimate(), "Drive instance requires a localizer attached to determine heading!"), angle, delta);
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive);
    }

    /**
     * Construct a new TurnTask that will turn the robot to the given angle.
     *
     * @param powerIn      the consumer to set the power of the robot, can ignore the x and y values if not needed
     * @param poseEstimate the supplier to get the current pose of the robot, can ignore the x and y values if not needed
     * @param angle        the angle to turn to, if this is a delta angle this will be counter-clockwise
     * @param delta        if this angle is a delta from the current drive rotation at runtime
     */
    public TurnTask(Consumer<PoseVelocity2d> powerIn, Supplier<Pose2d> poseEstimate, Measure<Angle> angle, boolean delta) {
        this.powerIn = powerIn;
        this.poseEstimate = poseEstimate;
        setDelta = delta;
        unmodifiedAngRad = Mathf.angleModulus(angle).in(Radians);
        // Sane defaults
        pidf = new PController(3);
        tolerance = Degrees.of(1);
    }

    /**
     * Construct a new TurnTask that will turn the robot to the given global angle.
     *
     * @param powerIn      the consumer to set the power of the robot, can ignore the x and y values if not needed
     * @param poseEstimate the supplier to get the current pose of the robot, can ignore the x and y values if not needed
     * @param angle        the angle to turn to in a global coordinate frame
     */
    public TurnTask(Consumer<PoseVelocity2d> powerIn, Supplier<Pose2d> poseEstimate, Measure<Angle> angle) {
        this(powerIn, poseEstimate, angle, false);
    }

    /**
     * Set the PIDF controller to use for turning.
     * By default, a P controller with a gain of 3 is used.
     *
     * @param pidf the PIDF controller to use
     * @return this
     */
    public TurnTask withPIDF(PIDF pidf) {
        this.pidf = pidf;
        return this;
    }

    /**
     * Set the tolerance for the turn task.
     *
     * @param tolerance the tolerance to use
     * @return this
     */
    public TurnTask withTolerance(Measure<Angle> tolerance) {
        if (tolerance.magnitude() <= 0) {
            throw new NotStrictlyPositiveException(tolerance.magnitude());
        }
        this.tolerance = tolerance;
        return this;
    }

    @Override
    protected void init() {
        angRad = setDelta
                ? Mathf.inputModulus(poseEstimate.get().heading.toDouble() + unmodifiedAngRad, -Math.PI, Math.PI)
                : unmodifiedAngRad;
    }

    @Override
    protected void periodic() {
        Pose2d pose = poseEstimate.get();
        double errRad = Mathf.inputModulus(pose.heading.toDouble(), -Math.PI, Math.PI) - angRad;
        powerIn.accept(Geometry.poseToVel(
                new Pose2d(0, 0, pidf.calculate(Mathf.inputModulus(errRad, -Math.PI, Math.PI), 0))
        ));

        Drawing.useCanvas(canvas -> {
            canvas.setStroke("#4CAF50");
            Drawing.drawRobot(canvas, new Pose2d(pose.position, angRad));
        });
    }

    @Override
    protected boolean isTaskFinished() {
        return Mathf.isNear(Mathf.inputModulus(poseEstimate.get().heading.toDouble(), -Math.PI, Math.PI), angRad, tolerance.in(Radians));
    }
}
