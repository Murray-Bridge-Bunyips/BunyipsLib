package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.PIDF;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.SystemController;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems.drive.Moveable;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.ForeverTask;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Drawing;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Geometry;

/**
 * TeleOp drive task to align to a global field coordinate using a localizer and PID.
 * Internally uses a feedforward based the direction of the point.
 *
 * @author Lucas Bubner, 2024
 * @since 4.1.0
 */
@Config
public class AlignToPointDriveTask extends ForeverTask {
    /**
     * The tolerance at which the robot will stop trying to align to a point if it is within this radius to the point.
     */
    public static double VECTOR_DELTA_CUTOFF_INCHES = 6;

    private final SystemController controller;
    private final Moveable drive;

    private final Supplier<Float> pX;
    private final Supplier<Float> pY;
    private final Supplier<Vector2d> pointSupplier;

    private double maxRotation = 1;
    private Vector2d lastPoint;
    private boolean fieldCentric;

    /**
     * Construct a new pass-through AlignToPointTask.
     *
     * @param passThroughPoseX   the pose X power to pass through to the drive
     * @param passThroughPoseY   the pose Y power to pass through to the drive
     * @param drive              drive instance to use, can optionally be a BunyipsSubsystem for auto-attachment
     * @param rotationController rotation PID controller to use
     * @param point              the point to align to in field space, will use the drive's pose estimate for current position
     */
    public AlignToPointDriveTask(@Nullable Supplier<Float> passThroughPoseX, @Nullable Supplier<Float> passThroughPoseY, Moveable drive, SystemController rotationController, Supplier<Vector2d> point) {
        this.drive = drive;
        if (drive instanceof BunyipsSubsystem)
            onSubsystem((BunyipsSubsystem) drive, false);
        pointSupplier = point;
        controller = rotationController;
        if (controller instanceof PIDF) {
            // Default tolerance is too low, will set minimum bound
            ((PIDF) controller).getPIDFController()
                    .setTolerance(Math.max(Math.toRadians(1), ((PIDF) controller).getPIDFController().getTolerance()[0]));
        }
        pX = passThroughPoseX;
        pY = passThroughPoseY;

        Vector2d currentTarget = point.get();
        withName("Align To Point: " + currentTarget);
        lastPoint = currentTarget;
    }

    /**
     * Construct a new controller-based pass-through AlignToPointTask.
     *
     * @param passThroughTranslation the controller where the left stick will be used to pass translation pose
     * @param rotationController     rotation PID controller to use
     * @param drive                  drive instance to use, can optionally be a BunyipsSubsystem for auto-attachment
     * @param point                  the point to align to in field space, will use the drive's pose estimate for current position
     */
    public AlignToPointDriveTask(Gamepad passThroughTranslation, Moveable drive, PIDF rotationController, Supplier<Vector2d> point) {
        this(() -> -passThroughTranslation.left_stick_y, () -> -passThroughTranslation.left_stick_x, drive, rotationController, point);
    }

    /**
     * Construct a turn-only AlignToPointTask.
     * <p>
     * This constructor will not permit active translation while the task is running, but is not recommended since
     * this is a TeleOp task with no end condition. Alternative solutions exist for aligning to a point in Autonomous
     * (e.g. by method of {@link TurnTask}, as this task is designed for dynamic conditions to continuous alignment).
     *
     * @param drive              drive instance to use, can optionally be a BunyipsSubsystem for auto-attachment
     * @param rotationController rotation PID controller to use
     * @param point              the point to align to in field space, will use the drive's pose estimate for current position
     */
    public AlignToPointDriveTask(Moveable drive, PIDF rotationController, Supplier<Vector2d> point) {
        this(null, null, drive, rotationController, point);
    }

    /**
     * Set a maximum magnitude that this task can turn at.
     *
     * @param speed the max magnitude from [-1.0, 1.0]
     * @return this
     */
    public AlignToPointDriveTask withMaxRotationSpeed(double speed) {
        maxRotation = Math.abs(speed);
        return this;
    }

    /**
     * Whether to use field centric control on the passed-through translation.
     *
     * @param enabled whether to use field centric control
     * @return this
     */
    public AlignToPointDriveTask withFieldCentric(boolean enabled) {
        fieldCentric = enabled;
        return this;
    }

    @Override
    protected void periodic() {
        // https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java
        Pose2d poseEstimate = drive.getPoseEstimate();
        if (poseEstimate == null)
            throw new IllegalStateException("AlignToPointDriveTask requires a localizer to be attached to the drive system!");
        Vector2d point = pointSupplier.get();
        if (!point.equals(lastPoint)) {
            // Dynamically update the name of the task to match
            withName("Align To Point: " + point);
        }
        lastPoint = point;

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(pX.get(), pY.get());
        Vector2d robotFrameInput = poseEstimate.heading.inverse().times(fieldFrameInput);

        // Difference between the target vector and the bot's position
        Vector2d difference = point.minus(poseEstimate.position);
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angleCast().toDouble();

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = Rotation2d.exp(-Math.PI / 2)
                .times(fieldFrameInput.unaryMinus()).dot(difference) / (difference.norm() * difference.norm());

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double headingInput = controller.calculate(poseEstimate.heading.toDouble(), theta) + thetaFF;
        headingInput = Mathf.clamp(headingInput, -maxRotation, maxRotation);

        // If we're at a discontinuity, we can't really do much so we should stop rotating
        if (Double.isNaN(headingInput) || Mathf.isNear(0, Geometry.distBetween(drive.getPoseEstimate().position, point), VECTOR_DELTA_CUTOFF_INCHES))
            headingInput = 0;

        // Combine the x/y velocity with our derived angular velocity
        drive.setPower(Geometry.poseToVel(new Pose2d(fieldCentric ? robotFrameInput : fieldFrameInput, headingInput)));

        // Draw the target on the field with lines to the target
        Drawing.useCanvas(canvas -> canvas.setStroke("#dd2c00")
                .strokeCircle(point.x, point.y, 2)
                .setStroke("#b89eff")
                .strokeLine(point.x, point.y, poseEstimate.position.x, poseEstimate.position.y)
                .setStroke("#ffce7a")
                .strokeLine(point.x, point.y, point.x, poseEstimate.position.y)
                .strokeLine(point.x, poseEstimate.position.y, poseEstimate.position.x, poseEstimate.position.y));
    }
}
